# Donanım

Ball & beam düzeneğinin fiziksel tarafı: parça listesi, doğrulanmış pin
haritası, kablolama tuzakları, bağlama geometrisi ve kamera sürücüsünün
kurulumu. Kararların gerekçeleri `docs/decision_log.md` içindedir; burada
sadece **çalışan** yapılandırma var.

## Parça listesi

| Parça | Ayrıntı | Not |
|---|---|---|
| Denetleyici | ESP32-S3-DevKitC-1 (N8, 8 MB flash, PSRAM yok) | İki USB kablosu: native USB-Serial-JTAG (veri) + UART köprüsü (konsol) |
| Step sürücü | TMC2208 (breakout) | UART modunda, 1/256 mikro adım, `IRUN=10` → 0,61 A rms |
| Motor | NEMA 17 (38 mm, 200 adım/tur) | Krankı doğrudan sürüyor, redüksiyon yok |
| Açı sensörü | AS5600 (I²C breakout) + mil ucunda mıknatıs | 4096 sayım/tur, 400 kHz |
| Konum sensörü | PS3 Eye kamera, 320×240 @ 100 fps | Topun konumu PC'de görüntüden çıkarılıyor |
| Yedek konum sensörü | HC-SR04 | Takılı ama kullanılmıyor (K-013) |
| Top | Turuncu pinpon topu | Renk eşiği `gui/calibration.json` içinde |
| Beam | ~45 cm izlenebilir yol, 477 mm mafsal–uç pim mesafesi | |
| Güç | 12 V (motor katı) + USB 5 V (lojik) | Ortak GND zorunlu |
| Basılı parçalar | `hardware/3d/` | Kaynak ve atıf: `hardware/3d/ATTRIBUTION.md` |

## Pin haritası (doğrulanmış)

Tek doğruluk kaynağı `firmware/src/board_pins.h`.

| Sinyal | ESP32-S3 | Notlar |
|---|---|---|
| STEP | GPIO4 | LEDC kare dalgası; aynı sinyal PCNT'ye geri besleniyor |
| DIR | GPIO5 | `1` → krank 12'ye doğru → top p2'ye doğru |
| TMC UART TX | GPIO6 | **1 kΩ seri direnç üzerinden** PDN_UART pinine |
| TMC UART RX | GPIO7 | Doğrudan PDN_UART pinine, dirençsiz |
| TMC EN | GPIO15 | LOW = sürücü etkin |
| AS5600 SDA | GPIO8 | Breakout'ta 10 kΩ pull-up var; ESP32 dahili pull-up kapalı |
| AS5600 SCL | GPIO9 | 400 kHz |
| **AS5600 DIR** | **GND** | **Harici kablo, zorunlu** — boşta bırakılırsa çip her okumada polariteyi değiştirir (K-018) |
| HC-SR04 TRIG | GPIO10 | 3,3 V sürüş yeterli |
| HC-SR04 ECHO | GPIO11 | 1k/2k bölücü (5 V → 3,33 V) |
| Döngü probu | GPIO16 | 4 ms zamanlayıcı ISR'ında toggle; jitter'ı analizörle ölçmek için |

TMC2208 tarafı: `CLK` → GND (dahili 12 MHz osilatör), `VIO` → 3V3,
`MS1`/`MS2` boşta (mikro adım UART'tan seçiliyor), `VM` → 12 V, sargılar
`M1A/M1B` ve `M2A/M2B`. AS5600'ün `OUT` ve `GPO` pinleri boşta.

### Kablolamada dikkat edilecekler

1. **Ortak GND:** ESP32, sürücü lojiği, sürücü motor katı ve 12 V güç
   kaynağı aynı toprakta olmalı.
2. **AS5600 DIR pini mutlaka GND'ye kablolanmalı.** Breakout'ta
   köprülenmiş görünse de köprülü değil; boşta kalırsa I²C hattının
   gürültüsünü alır ve aynı açıyı bir okumada `x`, diğerinde `4096−x`
   olarak bildirir.
3. **TMC UART tek hat half-duplex:** TX'e 1 kΩ, RX'e direnç yok. Yanlış
   yapılırsa ESP32'nin güçlü sürücüsü hattı sürücünün cevabına baskın
   gelir ve read-back hep `0xFF` döner.
4. **VIO ve VM farklı anlarda ayağa kalkabilir.** 12 V kapalıyken TMC
   UART'a cevap vermez; firmware bunu ölümcül saymaz, FAULT ile açılır ve
   besleme gelince `RUN` / `RESET_FAULT` tüm bring-up'ı yeniden dener.
5. **Mıknatıs merkezlemesi kritik:** AS5600 mıknatısı mil ekseninden
   kaçıksa açı okuması sinüzoidal hata yapar. Bu düzenekte kaçıklık
   `firmware/src/encoder_lut.h` içindeki düzeltme tablosuyla telafi
   ediliyor — **tablo bu oturuşa özeldir**, mıknatıs veya encoder kartı
   söküldüğünde yeniden üretilmelidir (`docs/encoder_calibration_*.csv`).

## Bağlama (linkage) geometrisi

Krank–biyel–beam düzeni. Ölçüler `firmware/src/config.h` içinde tek
kaynakta:

| Sembol | Değer | Anlamı |
|---|---|---|
| `r` | 31 mm | Motor mili merkezi → krank pimi |
| `l` | 90 mm | Biyel pim merkezleri arası |
| `d` | 477 mm | Beam mafsal pimi → beam ucundaki pim (ölçüldü) |
| pivot Δy | 90 mm | Mafsal pimi, motor milinin bu kadar üstünde |

`r` ve `l` referans 3D parçalardan da doğrulandı: `Crank.STL` 43 mm
uzunluğunda, 6 mm yarıçaplı iki uç → pim merkezleri arası 31 mm.
`Coupler.STL` (biyel) 114 mm, 12 mm yarıçaplı uçlar → 90 mm.

Dönüşüm doğrusal değil ve **simetrik de değil**: θ(φ) eğrisi φ = 75,5°'de
+3,265° ile tepe yapar, sonra geri döner. Bu yüzden komut açısı
%4 emniyet payıyla **±3,13°** ile sınırlı (`BEAM_THETA_MAX_DEG`), bu da
krank tarafında ±730 sayım demek (`CRANK_CMD_MAX_COUNTS`). Sınır aşılırsa
mekanizma tekilliğe girip beam'i geri indirir.

Çizimler: `hardware/drawings/linkage.svg` (ölçülü yan görünüş) ve
`hardware/drawings/theta_vs_phi.svg` (karakteristik + çalışma aralığı).

### Referans açı değerleri

| Sabit | Değer | Anlamı |
|---|---|---|
| `ENC_RAW_AT_6` | 535 | Krank saat 6 konumundayken ham encoder okuması |
| `ENC_LEVEL_COUNTS` | 1101 | Beam'in yatay olduğu krank açısı (sayım) |
| `ENC_SIGN_TOWARD_12` | −1 | Ham sayımın artış yönü ile 12 yönü arasındaki işaret |
| `ENC_START_WINDOW_COUNTS` | 400 | Açılışta devreye girmeye izin verilen pencere |

Bu değerler bu düzeneğe özgüdür; motor kablosu, krank veya encoder
söküldüyse yeniden ölçülmelidir.

## PS3 Eye sürücüsü (Windows)

DirectShow yolu kullanılamaz: çözünürlük isteğini yok sayıyor, pozlama ve
kazanç register'larına erişemiyor ve 1 saniyelik takılmalar yapıyor
(K-014). Çalışan yol **libusbK + pseyepy**:

1. Kamerayı tak. `tools/zadig-2.9.exe`'yi yönetici olarak çalıştır.
2. *Options → List All Devices* işaretli olsun.
3. Listeden **yalnızca `USB Camera-B4.09.24.1 (Interface 0)`** seç.
   Mikrofon arayüzü (`MI_01`) Windows'un kendi sürücüsüyle çalışıyor,
   ona **dokunma**.
4. Sürücüyü `libusbK` yap ve *Replace Driver*.
5. `pip install -r gui/requirements.txt` — `pseyepy` PyPI'da olmadığı için
   GitHub'dan C uzantısı derleniyor, MSVC Build Tools gerekir.

Çalışma noktası 320×240 @ 100 fps, pozlama 255, kazanç 62
(`gui/calibration.json`). Işık koşulları değişirse topun HSV eşiği
`gui/calibrate.py` ile yeniden ayarlanır.

## 3D parçalar

`hardware/3d/` altındaki STL'ler ve SolidWorks kaynak arşivi **bu projeye
ait değil**; IVProjects'in açık deposundan alındı. Kullanım koşulları için
`hardware/3d/ATTRIBUTION.md` dosyasını oku.
