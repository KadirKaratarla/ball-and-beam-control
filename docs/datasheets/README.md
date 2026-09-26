# Veri sayfaları

| Dosya | Parça | Kaynak |
|---|---|---|
| `esp32-s3_datasheet_en.pdf` | ESP32-S3 yongası | Espressif |
| `SCH_ESP32-S3-DevKitC-1_V1.1.pdf` | Geliştirme kartı şeması (V1.1) | Espressif |
| `TMC2202_TMC2208_TMC2224_datasheet_rev1.13.pdf` | TMC2208 step sürücü (rev 1.13) | Trinamic / ADI |
| `infineon-as5600-datasheet-en.pdf` | AS5600 manyetik açı sensörü | ams / Infineon |

Projede sık başvurulan bölümler:

* **TMC2208 §5** — UART çerçeve biçimi ve CRC8-ATM. Firmware her yazmayı
  geri okuyup doğruluyor (`firmware/src/tmc2208.c`).
* **TMC2208 §9** — akım ayarı:
  `I_RMS = ((CS+1)/32) × (V_FS / (R_SENSE + 30 mΩ)) × (1/√2)`.
  Bu düzenekte `IRUN = 10` → 0,61 A rms; daha yükseği sürücüyü gereksiz
  ısıtıyordu (K-026).
* **AS5600 §5.2** — `RAW_ANGLE` / `ANGLE` register'ları, 4096 sayım/tur.
  Firmware `RAW_ANGLE` okuyor (dahili filtre/histerezis yok).
* **AS5600 §7** — mıknatıs yerleşimi ve kaçıklık toleransı; bu düzenekteki
  kaçıklık yazılım tablosuyla telafi ediliyor (`encoder_lut.h`).
* **DevKitC-1 şeması** — hangi GPIO'ların USB-Serial-JTAG ve flash için
  ayrıldığı; pin haritası buna göre seçildi.

PS3 Eye kamerasının üretici veri sayfası yok. Kullanılan çalışma modu ve
sürücü kurulumu `hardware/README.md` içinde anlatıldı.
