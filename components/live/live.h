#pragma once
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/components/wifi/wifi_component.h"
#include "esphome/components/network/ip_address.h"
#include <memory>
#include <vector>
#include <functional>

#ifdef CONFIG_IDF_TARGET_ESP32P4
/* ------------------------------------------------------------------
 *  Bibliothèques matérielles – on inclut les headers « nouveaux ».
 * ------------------------------------------------------------------ */
#include "esp_jpeg_decoder.h"          // ← nouveau header JPEG
#include "esp_h264_decoder.h"          // ← nouveau header H.264
#include "esp_h264_types.h"            // définitions de structures
#include "ppa.h"
#include "esp_lcd_panel_ops.h"
#endif

/* ------------------------------------------------------------------
 *  Bibliothèques réseau (POSIX‑like) – toujours les mêmes.
 * ------------------------------------------------------------------ */
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

namespace esphome {
namespace live {

static const char *const TAG = "live";

/* ------------------------------------------------------------------
 *  Structure représentant une image décodée (indépendante du codec).
 * ------------------------------------------------------------------ */
struct VideoFrame {
  uint8_t *data{nullptr};
  size_t   size{0};
  uint16_t width{0};
  uint16_t height{0};
  uint32_t timestamp{0};
  bool     is_keyframe{false};
};

/* ------------------------------------------------------------------
 *  Classe principale du composant.
 * ------------------------------------------------------------------ */
class LiveComponent : public Component {
 public:
  /* -------------------------- Cycle de vie -------------------------- */
  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  /* -------------------------- Configuration ------------------------ */
  void set_rtsp_url(const std::string &url) { rtsp_url_ = url; }
  void set_buffer_size(uint32_t size) { buffer_size_ = size; }
  void set_timeout(uint32_t timeout) { timeout_ms_ = timeout; }
  void set_target_fps(uint8_t fps) { target_fps_ = fps; }
  void set_decode_format(const std::string &fmt) { decode_format_ = fmt; }

  /* -------------------------- Callbacks --------------------------- */
  void set_on_frame_callback(std::function<void(const VideoFrame &)> &&cb) {
    on_frame_callback_ = std::move(cb);
  }

#ifdef CONFIG_IDF_TARGET_ESP32P4
  void set_lcd_panel(esp_lcd_panel_handle_t panel) { lcd_panel_ = panel; }
#endif

  void set_display_area(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    display_x_ = x;
    display_y_ = y;
    display_w_ = w;
    display_h_ = h;
  }

  /* -------------------------- Contrôle du flux -------------------- */
  bool start_stream();
  void stop_stream();
  bool is_streaming() const { return streaming_; }
  uint32_t get_fps() const { return current_fps_; }

  void enable_hardware_decoding(bool enable) { use_hardware_decode_ = enable; }

 protected:
  /* -------------------------- Variables de config ----------------- */
  std::string rtsp_url_;
  std::string decode_format_{"h264"};
  uint32_t    buffer_size_{8192};
  uint32_t    timeout_ms_{5000};
  uint8_t     target_fps_{30};
  bool        streaming_{false};
  bool        use_hardware_decode_{true};
  uint32_t    current_fps_{0};

  /* -------------------------- Tampons ---------------------------- */
  std::unique_ptr<uint8_t[]> buffer_;        // socket raw data
  std::unique_ptr<uint8_t[]> decode_buffer_; // décodage (YUV/RGB)

#ifdef CONFIG_IDF_TARGET_ESP32P4
  /* -------------------------- Décodeurs matériels ---------------- */
  esp_h264_dec_handle_t h264_decoder_{nullptr};
  jpeg_dec_handle_t    jpeg_decoder_{nullptr};
  ppa_client_handle_t  ppa_client_{nullptr};

  /* -------------------------- Panel LCD -------------------------- */
  esp_lcd_panel_handle_t lcd_panel_{nullptr};
#endif

  /* -------------------------- Zone d’affichage ------------------ */
  uint16_t display_x_{0}, display_y_{0}, display_w_{320}, display_h_{240};

  /* -------------------------- Callback utilisateur --------------- */
  std::function<void(const VideoFrame &)> on_frame_callback_;

  /* -------------------------- RTSP / RTP -------------------------- */
  bool connect_rtsp();
  void disconnect_rtsp();
  bool send_rtsp_request(const std::string &req);
  std::string receive_rtsp_response();
  void process_rtp_stream();

  /* -------------------------- Décodage --------------------------- */
  bool init_hardware_decoders();
  void cleanup_hardware_decoders();
  bool decode_jpeg_frame(const uint8_t *data, size_t len, VideoFrame &frame);
  bool decode_h264_frame(const uint8_t *data, size_t len, VideoFrame &frame);

  /* -------------------------- Affichage -------------------------- */
  void display_frame_direct(const VideoFrame &frame);
  bool scale_frame_with_ppa(const VideoFrame &in, VideoFrame &out);

  /* -------------------------- Statistiques ----------------------- */
  uint32_t frame_count_{0};
  uint32_t last_fps_calc_{0};

  /* -------------------------- RTP ------------------------------- */
  uint16_t last_sequence_{0};

  /* -------------------------- RTSP state ------------------------ */
  std::string session_id_;
  uint16_t    client_port_{8000};
  uint16_t    server_port_{0};
  uint32_t    sequence_number_{0};

  /* -------------------------- Sockets --------------------------- */
  int rtsp_socket_{-1};
  int rtp_socket_{-1};
};

}  // namespace live
}  // namespace esphome
