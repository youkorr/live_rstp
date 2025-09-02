# Exemples d’utilisation du composant LIVE avec ESP32-P4

## 1. Caméra IP H.264 haute résolution

```yaml
# Configuration pour caméra IP Hikvision/Dahua
live:
  id: hd_camera
  rtsp_url: "rtsp://admin:password@192.168.1.100:554/h264Preview_01_main"
  decode_format: "h264"
  target_fps: 25
  buffer_size: 32768
  enable_hardware_decode: true
  
  display_area:
    x: 0
    y: 0  
    width: 800
    height: 600
    
  on_frame:
    - lambda: |-
        ESP_LOGD("camera", "Frame: %dx%d, %d bytes, keyframe: %s", 
                 frame.width, frame.height, frame.size,
                 frame.is_keyframe ? "yes" : "no");
```

## 2. Caméra MJPEG pour performance

```yaml
# Configuration pour caméra MJPEG (plus simple, moins de bande passante)
live:
  id: mjpeg_camera
  rtsp_url: "rtsp://user:pass@192.168.1.101:554/mjpeg"
  decode_format: "jpeg"
  target_fps: 15  # Plus bas pour économiser la bande passante
  buffer_size: 16384
  
  display_area:
    x: 100
    y: 100
    width: 640
    height: 480

  on_frame:
    - homeassistant.service:
        service: camera.snapshot
        data:
          entity_id: camera.esp32_live_feed
```

## 3. Multi-caméras avec affichage en mosaïque

```yaml
# Configuration pour 4 caméras en mosaïque
live:
  - id: camera_1
    rtsp_url: "rtsp://admin:pass@192.168.1.100:554/stream1"
    decode_format: "h264"
    display_area: {x: 0, y: 0, width: 400, height: 300}
    target_fps: 15
    
  - id: camera_2  
    rtsp_url: "rtsp://admin:pass@192.168.1.101:554/stream1"
    decode_format: "h264"
    display_area: {x: 400, y: 0, width: 400, height: 300}
    target_fps: 15
    
  - id: camera_3
    rtsp_url: "rtsp://admin:pass@192.168.1.102:554/stream1" 
    decode_format: "h264"
    display_area: {x: 0, y: 300, width: 400, height: 300}
    target_fps: 15
    
  - id: camera_4
    rtsp_url: "rtsp://admin:pass@192.168.1.103:554/stream1"
    decode_format: "h264" 
    display_area: {x: 400, y: 300, width: 400, height: 300}
    target_fps: 15

# Contrôle global des caméras
switch:
  - platform: template
    name: "All Cameras"
    turn_on_action:
      - lambda: |-
          id(camera_1)->start_stream();
          id(camera_2)->start_stream(); 
          id(camera_3)->start_stream();
          id(camera_4)->start_stream();
    turn_off_action:
      - lambda: |-
          id(camera_1)->stop_stream();
          id(camera_2)->stop_stream();
          id(camera_3)->stop_stream();
          id(camera_4)->stop_stream();
```

## 4. Intégration avec détection de mouvement

```yaml
live:
  id: security_camera
  rtsp_url: "rtsp://admin:pass@192.168.1.100:554/stream"
  decode_format: "h264"
  target_fps: 30
  
  on_frame:
    - lambda: |-
        static uint32_t last_motion_check = 0;
        static std::vector<uint8_t> prev_frame;
        
        uint32_t now = millis();
        if (now - last_motion_check > 1000) {  // Vérifier chaque seconde
          
          if (!prev_frame.empty()) {
            // Simple détection de mouvement par différence de pixels
            uint32_t diff_pixels = 0;
            uint32_t total_pixels = frame.width * frame.height;
            
            for (uint32_t i = 0; i < total_pixels && i < prev_frame.size(); i++) {
              if (abs(frame.data[i] - prev_frame[i]) > 30) {
                diff_pixels++;
              }
            }
            
            float motion_percent = (float)diff_pixels / total_pixels * 100;
            
            if (motion_percent > 5.0) {  // 5% de pixels différents
              ESP_LOGI("motion", "Motion detected: %.2f%%", motion_percent);
              
              // Déclencher une alerte
              id(motion_detected)->publish_state(true);
            }
          }
          
          // Sauvegarder la frame actuelle
          prev_frame.assign(frame.data, frame.data + frame.size);
          last_motion_check = now;
        }

binary_sensor:
  - platform: template
    name: "Motion Detected"
    id: motion_detected
    auto_off: 10s  # Se remet à off après 10 secondes
```

## 5. Enregistrement de snapshots

```yaml
live:
  id: snapshot_camera
  rtsp_url: "rtsp://admin:pass@192.168.1.100:554/stream"
  
  on_frame:
    - if:
        condition:
          lambda: return id(save_snapshot).state;
        then:
          - lambda: |-
              // Sauvegarder le snapshot (implémentation simplifiée)
              char filename[50];
              sprintf(filename, "/snapshot_%lu.jpg", millis());
              
              // Ici, il faudrait implémenter la sauvegarde sur SD card
              // ou l'envoi vers un serveur
              ESP_LOGI("snapshot", "Saving snapshot: %s", filename);
              
              id(save_snapshot)->turn_off();

switch:
  - platform: template
    name: "Save Snapshot"
    id: save_snapshot
    turn_on_action:
      - delay: 100ms  # Le snapshot sera pris dans on_frame
```

## 6. Streaming avec audio (si supporté)

```yaml
live:
  id: av_stream
  rtsp_url: "rtsp://admin:pass@192.168.1.100:554/av_stream"
  decode_format: "h264"
  enable_audio: true  # Feature à implémenter
  
  on_audio_frame:
    - lambda: |-
        // Traitement audio (nécessiterait extension du composant)
        ESP_LOGV("audio", "Audio frame: %d samples", audio_frame.samples);
```

## 7. Configuration pour performance optimale

```yaml
# Optimisations pour ESP32-P4
esphome:
  platformio_options:
    # Optimisations mémoire  
    build_flags:
      - -DCONFIG_ESP32_DEFAULT_CPU_FREQ_240=1
      - -DCONFIG_ESP32_SPIRAM_SUPPORT=1
      - -DCONFIG_SPIRAM_USE_CAPS_ALLOC=1
      - -DCONFIG_SPIRAM_USE_MALLOC=1
    
    # Partitions personnalisées pour plus d'espace
    board_build.partitions: custom_partitions.csv

live:
  id: optimized_stream
  rtsp_url: "rtsp://camera/stream"
  
  # Configuration optimisée
  buffer_size: 65536  # Buffer plus large
  target_fps: 30
  enable_hardware_decode: true
  
  # Zone d'affichage optimisée
  display_area:
    width: 800
    height: 600
    
sensor:
  # Monitoring des performances
  - platform: template
    name: "CPU Usage"
    lambda: |-
      TaskHandle_t idle_task = xTaskGetIdleTaskHandle();
      return 100.0 - (uxTaskGetStackHighWaterMark(idle_task) / 1000.0);
    unit_of_measurement: "%"
```

## Notes importantes

- **Mémoire** : Le décodage vidéo HD consomme beaucoup de RAM, utilisez la PSRAM
- **Performance** : L’ESP32-P4 peut gérer du 1080p@30fps avec les accélérateurs hardware
- **Formats** : H.264 offre la meilleure qualité/bande passante, MJPEG est plus simple
- **Réseau** : Une connexion WiFi stable est critique pour le streaming
- **Température** : Surveillez la température, le décodage vidéo chauffe le processeur
