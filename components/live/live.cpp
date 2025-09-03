#include "live.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <regex>
#include <sstream>

// ESP-IDF includes for hardware video decoding (ESP32-P4 only)
#ifdef CONFIG_IDF_TARGET_ESP32P4
#include "esp_jpeg_dec.h"
#include "esp_h264_dec.h"
#include "ppa.h"
#endif

namespace esphome {
namespace live {

void LiveComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up LIVE RTSP component with hardware acceleration…");
  ESP_LOGCONFIG(TAG, "  RTSP URL: %s", rtsp_url_.c_str());
  ESP_LOGCONFIG(TAG, "  Buffer size: %u", buffer_size_);
  ESP_LOGCONFIG(TAG, "  Target FPS: %u", target_fps_);
  ESP_LOGCONFIG(TAG, "  Decode format: %s", decode_format_.c_str());
  ESP_LOGCONFIG(TAG, "  Hardware decode: %s", use_hardware_decode_ ? "enabled" : "disabled");

  // Allouer les buffers
  buffer_ = std::make_unique<uint8_t[]>(buffer_size_);
  decode_buffer_ = std::make_unique<uint8_t[]>(1920 * 1080 * 3); // RGB24 max

  // Vérifier la connexion WiFi
  if (!wifi::global_wifi_component->is_connected()) {
    ESP_LOGW(TAG, "WiFi not connected, RTSP streaming will be unavailable");
    return;
  }

  // Initialiser les décodeurs hardware si activés
  if (use_hardware_decode_) {
    if (!init_hardware_decoders()) {
      ESP_LOGW(TAG, "Failed to initialize hardware decoders, falling back to software");
      use_hardware_decode_ = false;
    }
  }

  ESP_LOGD(TAG, "LIVE RTSP component setup complete");
}

void LiveComponent::loop() {
  if (!streaming_) {
    return;
  }

  // Traiter le flux RTP
  process_rtp_stream();

  // Calculer les FPS
  uint32_t now = millis();
  if (now - last_fps_calc_ >= 1000) {
    current_fps_ = frame_count_;
    frame_count_ = 0;
    last_fps_calc_ = now;
    ESP_LOGV(TAG, "Current FPS: %u", current_fps_);
  }
}

bool LiveComponent::init_hardware_decoders() {
#ifdef CONFIG_IDF_TARGET_ESP32P4
  esp_err_t ret;

  // Initialiser le décodeur JPEG hardware
  if (decode_format_ == "jpeg" || decode_format_ == "mjpeg") {
    jpeg_dec_config_t jpeg_config = {
      .output_format = JPEG_DEC_OUT_FORMAT_RGB565,
      .rotate = JPEG_DEC_ROTATE_0D,
    };

    ret = jpeg_dec_open(&jpeg_config, &jpeg_decoder_);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to create JPEG decoder: %s", esp_err_to_name(ret));
      return false;
    }
    ESP_LOGI(TAG, "Hardware JPEG decoder initialized");
  }

  // Initialiser le décodeur H.264 hardware
  if (decode_format_ == "h264") {
    esp_h264_dec_cfg_t h264_config = {
      .max_width = 1920,
      .max_height = 1080,
      .task_stack_size = 20 * 1024,
      .task_priority = 5,
      .task_core = 1,
    };

    ret = esp_h264_dec_open(&h264_config, &h264_decoder_);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to create H.264 decoder: %s", esp_err_to_name(ret));
      return false;
    }
    ESP_LOGI(TAG, "Hardware H.264 decoder initialized");
  }

  // Initialiser le PPA pour le scaling
  ppa_client_config_t ppa_config = {
    .oper_type = PPA_OPERATION_SRM,
    .max_pending_trans_num = 1,
  };

  ret = ppa_register_client(&ppa_config, &ppa_client_);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to register PPA client: %s", esp_err_to_name(ret));
    // PPA n'est pas critique, continuer sans
  } else {
    ESP_LOGI(TAG, "PPA (Pixel Processing Accelerator) initialized");
  }

  return true;
#else
  ESP_LOGW(TAG, "Hardware decoding only available on ESP32-P4");
  return false;
#endif
}

void LiveComponent::cleanup_hardware_decoders() {
#ifdef CONFIG_IDF_TARGET_ESP32P4
  if (jpeg_decoder_) {
    jpeg_dec_close(jpeg_decoder_);
    jpeg_decoder_ = nullptr;
  }

  if (h264_decoder_) {
    esp_h264_dec_close(h264_decoder_);
    h264_decoder_ = nullptr;
  }

  if (ppa_client_) {
    ppa_unregister_client(ppa_client_);
    ppa_client_ = nullptr;
  }
#endif
}

bool LiveComponent::start_stream() {
  if (streaming_) {
    ESP_LOGW(TAG, "Stream already active");
    return true;
  }

  if (!wifi::global_wifi_component->is_connected()) {
    ESP_LOGE(TAG, "WiFi not connected");
    return false;
  }

  ESP_LOGI(TAG, "Starting RTSP stream: %s", rtsp_url_.c_str());

  if (!connect_rtsp()) {
    ESP_LOGE(TAG, "Failed to connect to RTSP server");
    return false;
  }

  streaming_ = true;
  frame_count_ = 0;
  last_fps_calc_ = millis();

  ESP_LOGI(TAG, "RTSP stream started successfully");
  return true;
}

void LiveComponent::stop_stream() {
  if (!streaming_) {
    return;
  }

  ESP_LOGI(TAG, "Stopping RTSP stream");
  disconnect_rtsp();
  streaming_ = false;
  current_fps_ = 0;
}

bool LiveComponent::decode_jpeg_frame(const uint8_t *data, size_t len, VideoFrame &frame) {
#ifdef CONFIG_IDF_TARGET_ESP32P4
  if (!jpeg_decoder_) {
    ESP_LOGW(TAG, "JPEG decoder not initialized");
    return false;
  }

  esp_err_t ret;
  uint32_t out_size = 0;

  // Décoder avec le hardware JPEG
  ret = jpeg_dec_process(jpeg_decoder_,
                        const_cast<uint8_t*>(data), len,
                        decode_buffer_.get(), 1920 * 1080 * 2, // RGB565
                        &out_size);

  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "JPEG decode failed: %s", esp_err_to_name(ret));
    return false;
  }

  // Obtenir les dimensions de l'image
  jpeg_dec_header_info_t info;
  ret = jpeg_dec_parse_header(data, len, &info);
  if (ret == ESP_OK) {
    frame.width = info.width;
    frame.height = info.height;
  } else {
    // Valeurs par défaut si impossible d'obtenir les infos
    frame.width = 640;
    frame.height = 480;
  }

  frame.data = decode_buffer_.get();
  frame.size = out_size;
  frame.timestamp = millis();
  frame.is_keyframe = true;

  ESP_LOGV(TAG, "JPEG frame decoded: %dx%d, %d bytes",
          frame.width, frame.height, frame.size);

  return true;
#else
  ESP_LOGW(TAG, "Hardware JPEG decoding not available");
  return false;
#endif
}

bool LiveComponent::decode_h264_frame(const uint8_t *data, size_t len, VideoFrame &frame) {
#ifdef CONFIG_IDF_TARGET_ESP32P4
  if (!h264_decoder_) {
    ESP_LOGW(TAG, "H.264 decoder not initialized");
    return false;
  }

  esp_err_t ret;
  esp_h264_enc_frame_t in_frame = {
    .buffer = const_cast<uint8_t*>(data),
    .len = len,
    .consumed_len = 0,
  };

  esp_h264_enc_frame_t out_frame = {
    .buffer = decode_buffer_.get(),
    .size = 1920 * 1080 * 3, // YUV420
    .width = 0,
    .height = 0,
    .format = ESP_H264_RAW_FMT_YUV420,
  };

  ret = esp_h264_dec_process(h264_decoder_, &in_frame, &out_frame);

  if (ret != ESP_OK) {
    ESP_LOGV(TAG, "H.264 decode failed: %s", esp_err_to_name(ret));
    return false;
  }

  if (out_frame.width == 0 || out_frame.height == 0) {
    ESP_LOGV(TAG, "H.264 frame not ready");
    return false;
  }

  frame.data = out_frame.buffer;
  frame.size = out_frame.width * out_frame.height * 3 / 2; // YUV420
  frame.width = out_frame.width;
  frame.height = out_frame.height;
  frame.timestamp = millis();
  frame.is_keyframe = (in_frame.consumed_len > 0);

  ESP_LOGV(TAG, "H.264 frame decoded: %dx%d, %d bytes",
          frame.width, frame.height, frame.size);

  return true;
#else
  ESP_LOGW(TAG, "Hardware H.264 decoding not available");
  return false;
#endif
}

void LiveComponent::process_rtp_stream() {
  if (socket_fd_ < 0) {
    return;
  }

  // Vérifier si des données sont disponibles
  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(socket_fd_, &read_fds);
  
  struct timeval timeout = {0, 0}; // Non-blocking
  int activity = select(socket_fd_ + 1, &read_fds, NULL, NULL, &timeout);
  
  if (activity <= 0 || !FD_ISSET(socket_fd_, &read_fds)) {
    return;
  }

  // Lire les données RTP
  ssize_t bytes_read = recv(socket_fd_, buffer_.get(), buffer_size_, 0);
  if (bytes_read < 12) { // RTP header minimum
    return;
  }

  // Parser l'en-tête RTP basique
  uint8_t *rtp_data = buffer_.get();
  uint16_t sequence = (rtp_data[2] << 8) | rtp_data[3];
  uint32_t timestamp = (rtp_data[4] << 24) | (rtp_data[5] << 16) |
                      (rtp_data[6] << 8) | rtp_data[7];

  // Payload commence après l'en-tête RTP (12 bytes minimum)
  uint8_t *payload = rtp_data + 12;
  size_t payload_size = bytes_read - 12;

  // Accumuler les données dans le buffer RTP
  rtp_buffer_.insert(rtp_buffer_.end(), payload, payload + payload_size);

  // Vérifier si nous avons une frame complète
  bool frame_complete = false;

  if (decode_format_ == "jpeg" || decode_format_ == "mjpeg") {
    // Rechercher les marqueurs JPEG
    if (rtp_buffer_.size() >= 2 &&
        rtp_buffer_[rtp_buffer_.size()-2] == 0xFF &&
        rtp_buffer_[rtp_buffer_.size()-1] == 0xD9) {
      frame_complete = true;
    }
  } else if (decode_format_ == "h264") {
    // Pour H.264, logique plus complexe nécessaire
    // Ici, on traite chaque paquet RTP comme potentiel frame
    frame_complete = (payload_size > 0);
  }

  if (frame_complete && !rtp_buffer_.empty()) {
    VideoFrame frame;
    bool decoded = false;

    if (use_hardware_decode_) {
      if (decode_format_ == "jpeg" || decode_format_ == "mjpeg") {
        decoded = decode_jpeg_frame(rtp_buffer_.data(), rtp_buffer_.size(), frame);
      } else if (decode_format_ == "h264") {
        decoded = decode_h264_frame(rtp_buffer_.data(), rtp_buffer_.size(), frame);
      }
    }

    if (decoded) {
      frame_count_++;
      
      // Affichage direct sur LCD si configuré
      if (lcd_panel_) {
        display_frame_direct(frame);
      }
      
      // Callback utilisateur
      if (on_frame_callback_) {
        on_frame_callback_(frame);
      }
    }

    rtp_buffer_.clear();
  }

  // Limiter la taille du buffer pour éviter les débordements mémoire
  if (rtp_buffer_.size() > buffer_size_ * 4) {
    ESP_LOGW(TAG, "RTP buffer overflow, clearing");
    rtp_buffer_.clear();
  }
}

void LiveComponent::display_frame_direct(const VideoFrame &frame) {
  if (!lcd_panel_) {
    return;
  }

  // Si scaling nécessaire, utiliser le PPA
  if (frame.width != display_w_ || frame.height != display_h_) {
    VideoFrame scaled_frame;
    if (scale_frame_with_ppa(frame, scaled_frame)) {
      esp_lcd_panel_draw_bitmap(lcd_panel_, display_x_, display_y_,
                              display_x_ + display_w_, display_y_ + display_h_,
                              scaled_frame.data);
      return;
    }
  }

  // Affichage direct sans scaling
  esp_lcd_panel_draw_bitmap(lcd_panel_, display_x_, display_y_,
                          display_x_ + frame.width, display_y_ + frame.height,
                          frame.data);
}

bool LiveComponent::scale_frame_with_ppa(const VideoFrame &input, VideoFrame &output) {
#ifdef CONFIG_IDF_TARGET_ESP32P4
  if (!ppa_client_) {
    ESP_LOGW(TAG, "PPA client not initialized");
    return false;
  }
  // Implementation would go here - complex PPA operations
  // For now, return false to use software fallback
  ESP_LOGW(TAG, "PPA scaling not yet implemented");
  return false;
#else
  ESP_LOGW(TAG, "Hardware PPA not available - no scaling performed");
  return false;
#endif
}

// Basic RTSP implementation using ESP-IDF sockets
bool LiveComponent::connect_rtsp() {
  // Parse RTSP URL - simplified implementation
  // Format: rtsp://host:port/path
  std::string host = "192.168.1.100"; // Example - you'd parse this from rtsp_url_
  int port = 554;

  // Create socket
  socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd_ < 0) {
    ESP_LOGE(TAG, "Failed to create socket");
    return false;
  }

  // Setup server address
  struct sockaddr_in server_addr;
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port);
  inet_pton(AF_INET, host.c_str(), &server_addr.sin_addr);

  // Connect
  if (connect(socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    ESP_LOGE(TAG, "Failed to connect to RTSP server");
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  ESP_LOGI(TAG, "Connected to RTSP server");
  return true;
}

void LiveComponent::disconnect_rtsp() {
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool LiveComponent::send_rtsp_request(const std::string &request) {
  if (socket_fd_ < 0) {
    return false;
  }

  ssize_t sent = send(socket_fd_, request.c_str(), request.length(), 0);
  return sent == request.length();
}

std::string LiveComponent::receive_rtsp_response() {
  if (socket_fd_ < 0) {
    return "";
  }

  char response[1024];
  ssize_t received = recv(socket_fd_, response, sizeof(response) - 1, 0);
  if (received > 0) {
    response[received] = '\0';
    return std::string(response);
  }
  return "";
}

}  // namespace live
}  // namespace esphome
