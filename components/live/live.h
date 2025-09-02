#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/wifi/wifi_component.h"
#include "esphome/components/network/ip_address.h"
#include <memory>
#include <vector>
#include <functional>

// ESP-IDF includes for hardware video decoding
#include "esp_jpeg_dec.h"
#include "esp_h264_dec.h"
#include "esp_ppa.h"
#include "esp_lcd_panel_ops.h"

// ESP-IDF networking
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

namespace esphome {
namespace live {

static const char *const TAG = "live";

// Structure pour les frames vidéo
struct VideoFrame {
  uint8_t *data;
  size_t size;
  uint16_t width;
  uint16_t height;
  uint32_t timestamp;
  bool is_keyframe;
};

class LiveComponent : public Component {
 public:
  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  void set_rtsp_url(const std::string &url) { rtsp_url_ = url; }
  void set_buffer_size(uint32_t size) { buffer_size_ = size; }
  void set_timeout(uint32_t timeout) { timeout_ms_ = timeout; }
  void set_target_fps(uint8_t fps) { target_fps_ = fps; }
  void set_decode_format(const std::string &format) { decode_format_ = format; }

  // Callbacks pour les frames décodées
  void set_on_frame_callback(std::function<void(const VideoFrame &)> &&callback) {
    on_frame_callback_ = std::move(callback);
  }

  // Callbacks pour affichage direct sur LCD
  void set_lcd_panel(esp_lcd_panel_handle_t panel) { lcd_panel_ = panel; }
  void set_display_area(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    display_x_ = x; display_y_ = y; display_w_ = w; display_h_ = h;
  }

  bool start_stream();
  void stop_stream();
  bool is_streaming() const { return streaming_; }
  uint32_t get_fps() const { return current_fps_; }

  // Contrôle hardware
  void enable_hardware_decoding(bool enable) { use_hardware_decode_ = enable; }

 protected:
  std::string rtsp_url_;
  std::string decode_format_{"h264"}; // "h264" ou "jpeg"
  uint32_t buffer_size_{8192};
  uint32_t timeout_ms_{5000};
  uint8_t target_fps_{30};
  bool streaming_{false};
  bool use_hardware_decode_{true};
  uint32_t current_fps_{0};

  // Network client (using ESP-IDF socket instead of Arduino WiFiClient)
  int socket_fd_{-1};
  std::unique_ptr<uint8_t[]> buffer_;
  std::unique_ptr<uint8_t[]> decode_buffer_;

  // Hardware decoders (utilisant le composant esp_h264 officiel)
  esp_h264_dec_handle_t h264_decoder_{nullptr};

  // LCD display
  esp_lcd_panel_handle_t lcd_panel_{nullptr};
  uint16_t display_x_{0}, display_y_{0}, display_w_{320}, display_h_{240};

  std::function<void(const VideoFrame &)> on_frame_callback_;

  // RTSP protocol
  bool connect_rtsp();
  void disconnect_rtsp();
  bool send_rtsp_request(const std::string &request);
  std::string receive_rtsp_response();
  void process_rtp_stream();

  // Hardware video decoding
  bool init_hardware_decoders();
  void cleanup_hardware_decoders();
  bool decode_jpeg_frame(const uint8_t *data, size_t len, VideoFrame &frame);
  bool decode_h264_frame(const uint8_t *data, size_t len, VideoFrame &frame);

  // Display
  void display_frame_direct(const VideoFrame &frame);
  bool scale_frame_with_ppa(const VideoFrame &input, VideoFrame &output);

  // Performance monitoring
  uint32_t frame_count_{0};
  uint32_t last_fps_calc_{0};

  // RTP processing
  std::vector<uint8_t> rtp_buffer_;
  uint16_t last_sequence_{0};

  // Variables RTSP
  std::string session_id_;
  uint16_t client_port_{8000};
  uint16_t server_port_{0};
  uint32_t sequence_number_{0};
};

}  // namespace live
}  // namespace esphome
