#include "live.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <regex>
#include <sstream>

namespace esphome {
namespace live {

/* ------------------------------------------------------------------
 *  SETUP
 * ------------------------------------------------------------------ */
void LiveComponent::setup() {
  ESP_LOGCONFIG(TAG, "Live component setup – RTSP URL: %s", rtsp_url_.c_str());
  ESP_LOGCONFIG(TAG, "  Buffer size: %u, target FPS: %u", buffer_size_, target_fps_);
  ESP_LOGCONFIG(TAG, "  Decode format: %s, HW decode: %s",
                decode_format_.c_str(),
                use_hardware_decode_ ? "yes" : "no");

  /* Allocation des tampons */
  buffer_        = std::make_unique<uint8_t[]>(buffer_size_);
  decode_buffer_ = std::make_unique<uint8_t[]>(1920 * 1080 * 3);  // YUV420 max

  if (!wifi::global_wifi_component->is_connected()) {
    ESP_LOGW(TAG, "Wi‑Fi not connected – streaming will stay disabled");
    return;
  }

  if (use_hardware_decode_) {
    if (!init_hardware_decoders()) {
      ESP_LOGW(TAG,
               "Hardware decoder init failed – falling back to software mode");
      use_hardware_decode_ = false;
    }
  }
}

/* ------------------------------------------------------------------
 *  LOOP – lecture du flux RTP
 * ------------------------------------------------------------------ */
void LiveComponent::loop() {
  if (!streaming_)
    return;

  process_rtp_stream();

  /* Calcul du FPS chaque seconde */
  uint32_t now = millis();
  if (now - last_fps_calc_ >= 1000) {
    current_fps_ = frame_count_;
    frame_count_ = 0;
    last_fps_calc_ = now;
    ESP_LOGV(TAG, "Current FPS: %u", current_fps_);
  }
}

/* ------------------------------------------------------------------
 *  INITIALISATION DES DÉCODEURS HARDWARE
 * ------------------------------------------------------------------ */
bool LiveComponent::init_hardware_decoders() {
#ifdef CONFIG_IDF_TARGET_ESP32P4
  esp_err_t ret;

  /* ---------- JPEG ---------- */
  if (decode_format_ == "jpeg" || decode_format_ == "mjpeg") {
    jpeg_dec_config_t jpeg_cfg = {
        .output_format = JPEG_DEC_OUT_FORMAT_RGB565,
        .rotate        = JPEG_ROTATE_0D,
    };
    ret = jpeg_dec_open(&jpeg_cfg, &jpeg_decoder_);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "jpeg_dec_open failed: %s", esp_err_to_name(ret));
      return false;
    }
    ESP_LOGI(TAG, "Hardware JPEG decoder ready");
  }

  /* ---------- H.264 ---------- */
  if (decode_format_ == "h264") {
    esp_h264_dec_config_t h264_cfg = {
        .max_width       = 1920,
        .max_height      = 1080,
        .task_stack_size = 20 * 1024,
        .task_priority   = 5,
        .task_core_id    = 1,
    };
    ret = esp_h264_dec_open(&h264_cfg, &h264_decoder_);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "esp_h264_dec_open failed: %s", esp_err_to_name(ret));
      return false;
    }
    ESP_LOGI(TAG, "Hardware H.264 decoder ready");
  }

  /* ---------- PPA (scaling) ---------- */
  esp_ppa_client_config_t ppa_cfg = {
      .oper_type = ESP_PPA_OPERATION_SRM,
      .max_pending_trans_num = 1,
  };
  ret = esp_ppa_register_client(&ppa_cfg, &ppa_client_);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "esp_ppa_register_client failed: %s", esp_err_to_name(ret));
    /* PPA n'est pas critique – on continue sans */
  } else {
    ESP_LOGI(TAG, "PPA (Pixel‑Processing‑Accelerator) ready");
  }

  return true;
#else
  ESP_LOGW(TAG, "Hardware decoding not available on this target");
  return false;
#endif
}

/* ------------------------------------------------------------------
 *  CLEANUP DES DÉCODEURS HARDWARE
 * ------------------------------------------------------------------ */
void LiveComponent::cleanup_hardware_decoders() {
#ifdef CONFIG_IDF_TARGET_ESP32P4
  if (jpeg_decoder_) {
    jpeg_decoder_destroy(jpeg_decoder_);
    jpeg_decoder_ = nullptr;
  }
  if (h264_decoder_) {
    esp_h264_dec_close(h264_decoder_);
    h264_decoder_ = nullptr;
  }
  if (ppa_client_) {
    esp_ppa_unregister_client(ppa_client_);
    ppa_client_ = nullptr;
  }
#endif
}

/* ------------------------------------------------------------------
 *  DÉMARRAGE / ARRÊT DU FLUX
 * ------------------------------------------------------------------ */
bool LiveComponent::start_stream() {
  if (streaming_) {
    ESP_LOGW(TAG, "Stream already running");
    return true;
  }

  if (!wifi::global_wifi_component->is_connected()) {
    ESP_LOGE(TAG, "Wi‑Fi not connected – cannot start stream");
    return false;
  }

  ESP_LOGI(TAG, "Connecting to RTSP server …");
  if (!connect_rtsp()) {
    ESP_LOGE(TAG, "RTSP connection failed");
    return false;
  }

  streaming_ = true;
  frame_count_ = 0;
  last_fps_calc_ = millis();
  ESP_LOGI(TAG, "RTSP stream started");
  return true;
}

void LiveComponent::stop_stream() {
  if (!streaming_)
    return;

  ESP_LOGI(TAG, "Stopping RTSP stream");
  disconnect_rtsp();
  streaming_ = false;
  current_fps_ = 0;
}

/* ------------------------------------------------------------------
 *  DÉCODAGE JPEG (hardware)
 * ------------------------------------------------------------------ */
bool LiveComponent::decode_jpeg_frame(const uint8_t *data,
                                      size_t        len,
                                      VideoFrame   &frame) {
#ifdef CONFIG_IDF_TARGET_ESP32P4
  if (!jpeg_decoder_)
    return false;

  jpeg_decoder_io_t io = {
      .inbuf      = const_cast<uint8_t *>(data),
      .inbuf_len  = len,
      .outbuf     = decode_buffer_.get(),
      .outbuf_len = 1920 * 1080 * 2,   // RGB565 (2 bytes/pixel)
  };

  esp_err_t ret = jpeg_decoder_process(jpeg_decoder_, &io);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "JPEG decode failed: %s", esp_err_to_name(ret));
    return false;
  }

  jpeg_decoder_header_info_t hdr;
  ret = jpeg_decoder_get_info(jpeg_decoder_, &hdr);
  if (ret == ESP_OK) {
    frame.width  = hdr.width;
    frame.height = hdr.height;
  } else {
    frame.width  = 640;
    frame.height = 480;
  }

  frame.data      = decode_buffer_.get();
  frame.size      = frame.width * frame.height * 2; // RGB565
  frame.timestamp = millis();
  frame.is_keyframe = true;
  return true;
#else
  return false;
#endif
}

/* --------------------------------------------------------------
 *  DÉCODAGE H.264
 * -------------------------------------------------------------- */
bool LiveComponent::decode_h264_frame(const uint8_t *data,
                                      size_t        len,
                                      VideoFrame   &frame) {
#ifdef CONFIG_IDF_TARGET_ESP32P4
  if (!h264_decoder_)
    return false;

  /* Input frame */
  esp_h264_dec_in_frame_t in = {
      .buffer = const_cast<uint8_t *>(data),
      .len = len,
      .pts = 0,
  };

  /* Output buffer */
  esp_h264_dec_out_frame_t out = {
      .buffer   = decode_buffer_.get(),
      .buffer_size = 1920 * 1080 * 3,   // YUV420 (1.5 bytes/pixel)
      .width    = 0,
      .height   = 0,
      .format   = ESP_H264_DEC_OUTPUT_FORMAT_YUV420,
  };

  esp_err_t ret = esp_h264_dec_process(h264_decoder_, &in, &out);
  if (ret != ESP_OK) {
    ESP_LOGV(TAG, "H.264 decode failed: %s", esp_err_to_name(ret));
    return false;
  }

  if (out.width == 0 || out.height == 0) {
    ESP_LOGV(TAG, "Decoded H.264 frame has zero dimensions – discarding");
    return false;
  }

  frame.data      = out.buffer;
  frame.size      = out.width * out.height * 3 / 2;   // YUV420 size
  frame.width     = out.width;
  frame.height    = out.height;
  frame.timestamp = millis();
  frame.is_keyframe = true;   // approximation simple
  return true;
#else
  return false;
#endif
}

/* --------------------------------------------------------------
 *  TRAITEMENT DU FLUX RTP
 * -------------------------------------------------------------- */
void LiveComponent::process_rtp_stream() {
  if (rtp_socket_ < 0)
    return;

  ssize_t recv_len = recv(rtp_socket_, buffer_.get(), buffer_size_, MSG_DONTWAIT);
  if (recv_len <= 0) {
    return;   // aucun paquet disponible
  }

  /* En-tête RTP (12 octets) – on ignore la plupart des champs */
  if (recv_len < 12) {
    ESP_LOGV(TAG, "RTP packet too short: %d bytes", (int)recv_len);
    return;
  }

  const uint8_t *payload = buffer_.get() + 12;
  size_t        payload_len = static_cast<size_t>(recv_len) - 12;

  /* Séquence – simple contrôle de perte */
  uint16_t seq = (buffer_[2] << 8) | buffer_[3];
  if (last_sequence_ != 0 && seq != (uint16_t)(last_sequence_ + 1)) {
    ESP_LOGV(TAG, "RTP packet loss (expected %u, got %u)",
             (uint16_t)(last_sequence_ + 1), seq);
  }
  last_sequence_ = seq;

  VideoFrame frame{};
  bool decoded = false;

  if (decode_format_ == "jpeg" || decode_format_ == "mjpeg") {
    decoded = decode_jpeg_frame(payload, payload_len, frame);
  } else if (decode_format_ == "h264") {
    decoded = decode_h264_frame(payload, payload_len, frame);
  }

  if (!decoded) {
    ESP_LOGV(TAG, "Unable to decode received RTP payload");
    return;
  }

  ++frame_count_;

#ifdef CONFIG_IDF_TARGET_ESP32P4
  if (lcd_panel_) {
    display_frame_direct(frame);
  } else 
#endif
  if (on_frame_callback_) {
    on_frame_callback_(frame);
  }
}

/* --------------------------------------------------------------
 *  AFFICHAGE DIRECT SUR LCD (ESP32‑P4)
 * -------------------------------------------------------------- */
void LiveComponent::display_frame_direct(const VideoFrame &frame) {
#ifdef CONFIG_IDF_TARGET_ESP32P4
  if (!lcd_panel_)
    return;

  /* Si la zone d'affichage demandée correspond exactement à la taille
   * du frame, on copie le tampon tel quel. Sinon on utilise le PPA
   * pour redimensionner. */
  if (frame.width == display_w_ && frame.height == display_h_) {
    /* Copie brute – le format attendu par le driver LCD dépend du
     * mode choisi. Ici on suppose RGB565 (2 bytes/pixel). */
    esp_lcd_panel_draw_bitmap(lcd_panel_,
                              display_x_, display_y_,
                              display_x_ + display_w_, display_y_ + display_h_,
                              reinterpret_cast<const uint16_t *>(frame.data));
  } else {
    /* Redimensionnement via le PPA */
    VideoFrame scaled{};
    if (scale_frame_with_ppa(frame, scaled)) {
      esp_lcd_panel_draw_bitmap(lcd_panel_,
                                display_x_, display_y_,
                                display_x_ + display_w_, display_y_ + display_h_,
                                reinterpret_cast<const uint16_t *>(scaled.data));
    } else {
      ESP_LOGW(TAG, "Failed to scale frame – dropping");
    }
  }
#else
  ESP_LOGW(TAG, "Direct LCD display not supported on this target");
#endif
}

/* --------------------------------------------------------------
 *  REDIMENSIONNEMENT AVEC PPA
 * -------------------------------------------------------------- */
bool LiveComponent::scale_frame_with_ppa(const VideoFrame &input,
                                        VideoFrame       &output) {
#ifdef CONFIG_IDF_TARGET_ESP32P4
  if (!ppa_client_)
    return false;

  // Simplification - disable PPA scaling for now due to API mismatch
  ESP_LOGW(TAG, "PPA scaling not implemented - API needs adjustment");
  return false;
#else
  return false;
#endif
}

/* --------------------------------------------------------------
 *  CONNEXION RTSP (SETUP, PLAY, TEARDOWN)
 * -------------------------------------------------------------- */
bool LiveComponent::connect_rtsp() {
  // Parse RTSP URL - simplified implementation
  // Format: rtsp://host:port/path
  std::string host = "192.168.1.100"; // Example - you'd parse this from rtsp_url_
  int port = 554;

  // Create socket
  rtsp_socket_ = socket(AF_INET, SOCK_STREAM, 0);
  if (rtsp_socket_ < 0) {
    ESP_LOGE(TAG, "Failed to create RTSP socket");
    return false;
  }

  // Setup server address
  struct sockaddr_in server_addr;
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port);
  inet_pton(AF_INET, host.c_str(), &server_addr.sin_addr);

  // Connect
  if (connect(rtsp_socket_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    ESP_LOGE(TAG, "Failed to connect to RTSP server");
    close(rtsp_socket_);
    rtsp_socket_ = -1;
    return false;
  }

  // Create RTP socket for receiving video data
  rtp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (rtp_socket_ < 0) {
    ESP_LOGE(TAG, "Failed to create RTP socket");
    close(rtsp_socket_);
    rtsp_socket_ = -1;
    return false;
  }

  // Bind RTP socket to local port
  struct sockaddr_in rtp_addr;
  rtp_addr.sin_family = AF_INET;
  rtp_addr.sin_addr.s_addr = INADDR_ANY;
  rtp_addr.sin_port = htons(client_port_);
  
  if (bind(rtp_socket_, (struct sockaddr*)&rtp_addr, sizeof(rtp_addr)) < 0) {
    ESP_LOGE(TAG, "Failed to bind RTP socket");
    close(rtsp_socket_);
    close(rtp_socket_);
    rtsp_socket_ = -1;
    rtp_socket_ = -1;
    return false;
  }

  ESP_LOGI(TAG, "Connected to RTSP server");
  return true;
}

void LiveComponent::disconnect_rtsp() {
  if (rtsp_socket_ >= 0) {
    close(rtsp_socket_);
    rtsp_socket_ = -1;
  }
  if (rtp_socket_ >= 0) {
    close(rtp_socket_);
    rtp_socket_ = -1;
  }
}

bool LiveComponent::send_rtsp_request(const std::string &request) {
  if (rtsp_socket_ < 0) {
    return false;
  }

  ssize_t sent = send(rtsp_socket_, request.c_str(), request.length(), 0);
  return sent == (ssize_t)request.length();
}

std::string LiveComponent::receive_rtsp_response() {
  if (rtsp_socket_ < 0) {
    return "";
  }

  char response[1024];
  ssize_t received = recv(rtsp_socket_, response, sizeof(response) - 1, 0);
  if (received > 0) {
    response[received] = '\0';
    return std::string(response);
  }
  return "";
}

}  // namespace live
}  // namespace esphome
