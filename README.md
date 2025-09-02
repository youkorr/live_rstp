esphome:
name: esp32-p4-video-streamer
platform: ESP32
board: esp32-s3-devkitc-1
platformio_options:
board_build.flash_mode: qio
board_build.partitions: huge_app.csv
build_flags:
- -DCONFIG_ESP32P4_JPEG_DECODE_ACCELERATE
- -DCONFIG_ESP32P4_H264_DECODE_ACCELERATE
- -DCONFIG_ESP32P4_PPA_ACCELERATE

wifi:
ssid: “VotreSSID”  
password: “VotreMotDePasse”

logger:
level: INFO

api:
encryption:
key: “votre_cle_api”

ota:
password: “votre_mot_de_passe_ota”

external_components:

- source:
  type: local
  path: components

# Configuration SPI pour écran haute résolution

spi:

- id: display_spi
  clk_pin: GPIO14
  mosi_pin: GPIO13
  miso_pin: GPIO12
  frequency: 80MHz

# Écran LCD haute résolution pour la vidéo

display:

- platform: ili9xxx
  model: ili9488  # Écran 480x320
  spi_id: display_spi
  cs_pin: GPIO15
  dc_pin: GPIO2
  reset_pin: GPIO4
  rotation: 0
  id: video_display
  update_interval: never  # Mise à jour directe par le composant vidéo

# Configuration LVGL pour les contrôles uniquement

lvgl:
id: lvgl_display
displays:
- video_display
buffer_size: 25%  # Buffer réduit car vidéo gérée séparément
log_level: WARN
color_depth: 16

# Interface de contrôle overlay

pages:
- id: control_page
widgets:
- obj:
id: video_overlay
x: 0
y: 0
width: 480
height: 280
bg_opa: TRANSP  # Transparent pour la vidéo
border_width: 0

```
    - obj:
        id: control_panel
        x: 0
        y: 280
        width: 480
        height: 40
        bg_color: 0x000000
        bg_opa: COVER
        widgets:
          - btn:
              id: play_btn
              x: 10
              y: 5
              width: 60
              height: 30
              widgets:
                - label:
                    text: "▶"
                    align: center
              on_click:
                - lambda: |-
                    if (!id(rtsp_stream)->is_streaming()) {
                      id(rtsp_stream)->start_stream();
                    } else {
                      id(rtsp_stream)->stop_stream();
                    }
          
          - label:
              id: fps_label
              x: 80
              y: 15
              text: "0 FPS"
          
          - label:
              id: resolution_label
              x: 150
              y: 15
              text: "Déconnecté"
```

# Composant RTSP avec décodage hardware

live:
id: rtsp_stream
rtsp_url: “rtsp://admin:password@192.168.1.100:554/h264Preview_01_main”
buffer_size: 16384      # Buffer plus grand pour la vidéo HD
timeout: 10000
target_fps: 25          # FPS cible
decode_format: “h264”   # ou “jpeg” pour MJPEG

# Configuration hardware

enable_hardware_decode: true

# Zone d’affichage sur l’écran

display_area:
x: 0
y: 0
width: 480
height: 280

# Callbacks

on_frame:
- lambda: |-
// Mettre à jour les informations d’affichage
char fps_text[20];
sprintf(fps_text, “%d FPS”, id(rtsp_stream)->get_fps());
id(fps_label)->set_text(fps_text);

```
    char res_text[30];
    sprintf(res_text, "%dx%d", frame.width, frame.height);
    id(resolution_label)->set_text(res_text);
```

on_connect:
- lambda: |-
auto btn = id(play_btn);
btn->get_child(0)->set_text(“⏸”);
id(resolution_label)->set_text(“Connecté”);

on_disconnect:
- lambda: |-
auto btn = id(play_btn);
btn->get_child(0)->set_text(“▶”);
id(fps_label)->set_text(“0 FPS”);
id(resolution_label)->set_text(“Déconnecté”);

# Monitoring de performance

sensor:

- platform: template
  name: “Video FPS”
  lambda: return id(rtsp_stream)->get_fps();
  unit_of_measurement: “fps”
  update_interval: 1s
- platform: template
  name: “Free Heap”
  lambda: return ESP.getFreeHeap();
  unit_of_measurement: “B”
  update_interval: 5s
- platform: template
  name: “PSRAM Free”
  lambda: return ESP.getFreePsram();
  unit_of_measurement: “B”
  update_interval: 5s

# Contrôles via Home Assistant

switch:

- platform: template
  name: “RTSP Stream”
  id: stream_switch
  turn_on_action:
  - lambda: id(rtsp_stream)->start_stream();
    turn_off_action:
  - lambda: id(rtsp_stream)->stop_stream();
    lambda: return id(rtsp_stream)->is_streaming();

binary_sensor:

- platform: template
  name: “Stream Active”
  lambda: return id(rtsp_stream)->is_streaming();

# Mise à jour périodique de l’interface

interval:

- interval: 100ms
  then:
  - lambda: |-
    // Mise à jour des statistiques en temps réel
    if (id(rtsp_stream)->is_streaming()) {
    static uint32_t last_update = 0;
    uint32_t now = millis();
    
    ```
    if (now - last_update > 1000) {  // Mise à jour chaque seconde
      char fps_text[20];
      sprintf(fps_text, "%d FPS", id(rtsp_stream)->get_fps());
      id(fps_label)->set_text(fps_text);
      
      last_update = now;
    }
    ```
    
    }

# Script de démarrage automatique

script:

- id: auto_start_stream
  mode: single
  then:
  - delay: 5s  # Attendre que tout soit initialisé
  - lambda: |-
    ESP_LOGI(“main”, “Auto-starting RTSP stream…”);
    id(rtsp_stream)->start_stream();

# Démarrage automatique du stream au boot

on_boot:
priority: -100  # Exécuter après tout le reste
then:
- script.execute: auto_start_stream

# Gestion des erreurs et reconnexion automatique

time:

- platform: homeassistant
  id: ha_time
  on_time:
  
  # Vérification de santé toutes les 30 secondes
  - seconds: /30
    then:
    - lambda: |-
      static uint32_t last_fps_check = 0;
      static uint8_t no_frame_count = 0;
      
      uint32_t current_fps = id(rtsp_stream)->get_fps();
      
      if (id(rtsp_stream)->is_streaming() && current_fps == 0) {
      no_frame_count++;
      ESP_LOGW(“main”, “No frames received, count: %d”, no_frame_count);
      
      ```
      // Reconnexion après 3 vérifications sans frames (90 secondes)
      if (no_frame_count >= 3) {
        ESP_LOGI("main", "Reconnecting RTSP stream...");
        id(rtsp_stream)->stop_stream();
        delay(2000);
        id(rtsp_stream)->start_stream();
        no_frame_count = 0;
      }
      ```
      
      } else {
      no_frame_count = 0;  // Reset counter si on reçoit des frames
      }
