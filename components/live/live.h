#include "live.h"
#include "esphome/core/log.h"
#include <regex>
#include <sstream>

// Note: ESP32-P4 hardware decoder headers not available in current ESP-IDF version
// Hardware acceleration will be disabled for now

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
  ESP_LOGW(TAG, "Hardware decoders not available in current ESP-IDF version");
  ESP_LOGI(TAG, "Component will operate in software-only mode");
  return false;
}

void LiveComponent::cleanup_hardware_decoders() {
  // No hardware resources to clean up in software-only mode
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
  ESP_LOGW(TAG, "Hardware JPEG decoder not available - software decoding not implemented yet");
  return false;
}

bool LiveComponent::decode_h264_frame(const uint8_t *data, size_t len, VideoFrame &frame) {
  ESP_LOGW(TAG, "Hardware H.264 decoder not available - software decoding not implemented yet");
  return false;
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
  ESP_LOGW(TAG, "Hardware PPA not available - no scaling performed");
  return false;
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
