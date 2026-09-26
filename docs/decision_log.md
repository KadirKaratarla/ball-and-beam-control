# Karar Kaydı (Decision Log)

Bu dosya, projenin gidişatını kalıcı olarak etkileyen kararları ve bunların
gerekçelerini tutar. Kod okunarak anlaşılamayacak "neden böyle yaptık"
bilgisi buraya yazılır.

---

## K-001 — TMC2208'den santigrat cinsinden sıcaklık okunamaz
**Tarih:** 2026-08-22 · **Faz:** 0 · **Durum:** Kabul edildi

**Bağlam:** İlk plan sürücüden "sıcaklık verisi" almayı öngörüyordu.

**Karar:** Santigrat cinsinden analog sıcaklık okuma kapsam dışı bırakıldı.
Bunun yerine `DRV_STATUS` register'ındaki bayraklar kullanılacak.

**Gerekçe:** Datasheet bölüm 13.1'e göre çipte 4 seviyeli bir eşik
karşılaştırıcı var, analog sensör yok. Okunabilenler:
- `otpw` (aşırı ısınma ön uyarı) ve `ot` (aşırı ısınma kesme) — eşikleri
  `FACTORY_CONF.OTTRIM` belirler, varsayılan OT=143°C / OTPW=120°C
- `t120`, `t143`, `t150`, `t157` — hangi eşiklerin aşıldığını gösteren 4 bit,
  yani kaba bir sıcaklık aralığı çıkarılabilir ama kesin değer çıkarılamaz

Gerçek santigrat değeri gerekirse ayrı bir NTC/DS18B20 eklenmeli. Bu proje
kapsamında yapılmayacak.

---

## K-002 — Klasör adında `&` karakteri kullanılamaz
**Tarih:** 2026-08-22 · **Faz:** 0 · **Durum:** Uygulandı

**Karar:** Proje kök dizini `E:\projeler\ball&beam` → `E:\projeler\ball_beam`
olarak değiştirildi.

**Gerekçe:** ESP-IDF derleme zinciri (CMake, ninja, `ld.exe`) yolları Windows
`cmd.exe` üzerinden geçiriyor ve `&` komut ayracı olarak yorumlanıyor. Build
şu hatalarla kırılıyordu:
```
'beam' is not recognized as an internal or external command
ld.exe: cannot find @E:\projeler\ball: Invalid argument
CMake Error: include could not find requested file: /tools/cmake/project.cmake
```

**Sonuç:** Bu dizin altındaki hiçbir klasör/dosya adında `&`, boşluk veya
kabuk tarafından yorumlanan özel karakter kullanılmayacak.

---

## K-003 — PlatformIO board seçimi: N16R8 için `4d_systems_esp32s3_gen4_r8n16`
**Tarih:** 2026-08-22 · **Faz:** 0 · **Durum:** Uygulandı

**Bağlam:** Elimizdeki çip **ESP32-S3-N16R8** (16MB Quad flash + 8MB Octal
PSRAM). PlatformIO'nun kurulu board listesinde tam bu isimde bir tanım yok;
`esp32-s3-devkitc-1` tanımı N8 (8MB flash, PSRAM yok) varsayıyor.

**Karar:** `platformio.ini` içinde `board = 4d_systems_esp32s3_gen4_r8n16`
kullanılıyor. Ayrıca `sdkconfig.defaults` ile flash/PSRAM ayarları sabitlendi:
```ini
CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y
CONFIG_ESPTOOLPY_FLASHMODE_QIO=y
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_SPIRAM_SPEED_80M=y
```

**Gerekçe:** Bu, kurulu tanımlar arasında donanımımızla birebir eşleşen
(R8N16 = 8MB Octal PSRAM + 16MB flash) ve `upload.flash_size` değeri zaten
16MB olan tek tanım. Üretici adının 4D Systems olması önemsiz —
`framework = espidf` kullandığımız için bu dosyadan üreticiye özel hiçbir
pin haritası çekilmiyor.

**Doğrulama:** Boot log'unda `SPI Flash Size : 16MB`, `Found 8MB PSRAM
device`, `SPI SRAM memory test OK`.

---

## K-004 — TMC2208 modülünde PDN_UART header pini fabrika çıkışı bağlı değil
**Tarih:** 2026-08-22 · **Faz:** 1.3 · **Durum:** Çözüldü (lehim yapıldı)

**Bağlam:** UART haberleşmesi hiçbir şekilde cevap vermiyordu. Kartın
silkscreen'inde ayrı ayrı **"PDN"** ve **"UART"** yazan iki pin var, ancak
multimetre ile aralarında süreklilik yok.

**Kök neden:** SilentStepStick ve klonlarında çipin `PDN_UART` bacağı,
kartın **alt yüzündeki lehim pad'leri** köprülenene kadar hiçbir header
pinine bağlı değildir. Ortadaki pad çipe, yanlardaki iki pad ise en yakın
header pinine gider. Bu yüzden "PDN" ve "UART" pinleri birbirine de bağlı
görünmez — ikisi de çipten kopuktur.

**Karar:** Alt yüzdeki pad'ler lehimle köprülendi, **PDN** pini
PDN_UART hattı olarak kullanılıyor.

**Not:** Bu bilgi çip datasheet'inde yok (kart üreticisinin PCB kararı).
Kaynak: instructables.com/UART-This-Serial-Control-of-Stepper-Motors-With-th
adım 3.

---

## K-005 — UART için tek GPIO yerine ayrı TX/RX pinleri
**Tarih:** 2026-08-22 · **Faz:** 1.3 · **Durum:** Uygulandı

**Bağlam:** İlk tasarımda tek bir GPIO hem TX hem RX olarak kullanılıyordu
(`uart_set_pin(port, 6, 6, ...)`), 1kΩ direnç GPIO ile sürücü arasındaydı.

**Sorun:** ESP32'nin UART TX çıkışı **push-pull** ve boştayken hattı aktif
olarak HIGH sürüyor (~40Ω çıkış direnci). TMC cevap verirken hattı LOW'a
çekmeye çalıştığında 1kΩ ile ESP32'nin güçlü sürücüsü arasında gerilim
bölücü oluşuyor: **direncin sürücü tarafı LOW'a iner ama ESP32'nin gördüğü
pin HIGH kalır.** Yazma çalışır, okuma asla çalışmaz.

**Karar:** Klasik "Y-kablo" topolojisi:
- **TX (GPIO6)** → 1kΩ seri direnç → PDN_UART
- **RX (GPIO7)** → doğrudan PDN_UART (dirençsiz)

1kΩ, iki taraf aynı anda sürerse akımı sınırlamak için hâlâ gerekli.
RX direncin sürücü tarafını dinlediği için TMC'nin çektiği LOW'u görebiliyor.

---

## K-006 — EN pini mutlaka sürülmeli, boşta bırakılamaz
**Tarih:** 2026-08-22 · **Faz:** 1.3 · **Durum:** Uygulandı

**Bağlam:** UART tamamen çalışır hale geldikten, tüm register'lar doğru
yazılıp okunduktan ve STEP darbelerinin çipe ulaştığı (`stst=0`) teyit
edildikten sonra bile motor dönmüyordu.

**Kök neden:** `IOIN` register'ı `ENN=1` gösteriyordu — güç katı kapalıydı.
TMC220x'in ENN girişinde **dahili pull-down yok** (datasheet pin tablosunda
TMC220x'in ENN'i için `pd` işareti yok, sadece TMC2224'te var), bu yüzden
bağlanmayan pin havada kalıp HIGH okunuyor.

**Karar:** EN pini **GPIO15**'e bağlandı ve firmware tarafından açıkça
sürülüyor (yapılandırma sırasında HIGH = disable, sonra LOW = enable).

**Yan fayda:** Yazılımsal acil durdurma / motoru serbest bırakma imkanı.

---

## K-007 — Sürücü yapılandırması her power-on'da yazılacak + reset bekçisi
**Tarih:** 2026-08-23 · **Faz:** 1.3 → 2 · **Durum:** Karar verildi, henüz kodlanmadı

**Bağlam:** TMC2208 register'ları uçucu. Datasheet: *"All registers become
reset to 0 upon power up, unless otherwise noted."* Sadece OTP'ye yazılan
değerler kalıcı.

**Karar:**
1. Boot'ta `GCONF` / `CHOPCONF` / `IHOLD_IRUN` yazılacak, ardından `IFCNT`
   artışı ve register geri okuması ile **doğrulanacak**
2. Core1'deki tanı görevi (50-100ms periyot) her turda `GSTAT` okuyacak;
   `reset=1` görülürse yapılandırma **anında yeniden yazılacak** ve olay
   loglanıp GUI'ye bildirilecek (`GSTAT.reset` okununca temizlenen bir
   latch, tam bir bekçi köpeği olarak çalışıyor)
3. `reset` olayı sırasında kontrol döngüsü güvenli duruma alınacak (step
   üretimi durdurulacak), yapılandırma doğrulanmadan sürüşe devam edilmeyecek

**Gerekçe — sessiz ve tehlikeli bozulma modu:** Sürücü resetlenirse
`GCONF.mstep_reg_select` 0'a döner ve mikroadım çözünürlüğü MS1/MS2
pinlerinden okunmaya başlar. Bizim kartta bu pinler boşta (dahili pull-down
= `00`), bu da TMC220x için **1/8 mikroadım** demektir. Aynı sayıda STEP
darbesi motoru **32 kat fazla** döndürür. Kontrol döngüsü bunu fark etmez ve
beam kontrolden çıkar. Top-denge sistemi için kabul edilemez bir risk.

**Ek risk:** VIO (3.3V) ve VM (12V) farklı anlarda ayağa kalkabilir. ESP32
önce boot edip TMC'yi VM stabil olmadan yapılandırırsa ayarlar sessizce
kaybolur — bekçi köpeği bunu da yakalar.

---

## K-008 — OTP programlanmayacak
**Tarih:** 2026-08-22 · **Faz:** 1.3 · **Durum:** Kabul edildi

**Karar:** TMC2208'in OTP (One-Time Programmable) hafızasına hiçbir şey
yazılmayacak. Tüm yapılandırma çalışma anında UART üzerinden yapılacak.

**Gerekçe:** OTP bitleri geri döndürülemez — bir kez set edilen bit silikonda
kalıcı değişiklik yapar, temizlenemez. Yapılandırmayı UART'tan yazmak aynı
sonucu veriyor ve deneme-yanılmaya izin veriyor. K-007'deki bekçi köpeği
mekanizması OTP'nin sağlayacağı kalıcılık ihtiyacını da karşılıyor.

---

## K-009 — Step darbeleri donanımdan üretilecek, meşgul-bekleme ile değil
**Tarih:** 2026-08-23 · **Faz:** 2.4 · **Durum:** Karar verildi, henüz kodlanmadı

**Bağlam:** Faz 1.3 test kodu `esp_rom_delay_us()` ile meşgul-bekleme yaparak
STEP darbesi üretiyor. Ölçülen: 24µs periyot = 41.7 kHz = 49 RPM.

**Sorun:** 41.7 kHz'de 2ms'lik kontrol döngüsüne **83 mikroadım** düşüyor.
Meşgul-bekleme CPU'yu bloke ettiği için Faz 2'nin sabit periyotlu kontrol
görevini imkansız kılar.

**Karar:** Step üretimi **LEDC** (PWM birimi) ile donanıma devredilecek.
Frekans değiştirmek = hız değiştirmek; PID çıktısı zaten step/s cinsinden hız
olacağı için birebir örtüşüyor. Yön ayrı GPIO (DIR) ile verilecek.

**Opsiyon:** Üretilen darbeler **PCNT** birimine geri beslenerek komut edilen
pozisyon CPU yükü olmadan sayılabilir (Faz 3.4'teki adım kaybı tespitinde
AS5600 ile karşılaştırma için).

---

## K-010 — HC-SR04 kontrol döngüsünde bloklamayacak
**Tarih:** 2026-08-22 · **Faz:** 0 · **Durum:** Kabul edildi (Faz 1.4'te doğrulanacak)

**Karar:** Mesafe ölçümü ayrı, asenkron bir görevde (timer input-capture /
interrupt ile echo darbe genişliği ölçümü) yapılacak. Kontrol döngüsü her
turda **son geçerli mesafe değerini** kullanacak, ölçümü beklemeyecek.

**Gerekçe:** Trigger→echo süresi mesafeye bağlı olarak ~150µs–25ms arasında
değişir (sesin gidiş-dönüş süresi). 2ms'lik döngü içinde bloklayarak okumak
döngüyü öldürür.

**Not:** Ölçüm gecikmesi kontrol gecikmesine dahil olacak, PID tuning'i
etkileyecek (Faz 6'da hesaba katılacak).

**Güncelleme (2026-09-13):** Asenkron ölçüm doğrulandı (tetikleme 16 µs
bloklıyor, echo kesmede ölçülüyor). Ancak HC-SR04 asıl sensör olmaktan
çıktı — bkz. K-012, K-013.

---

## K-011 — AS5600 I2C hızı 400 kHz
**Tarih:** 2026-09-01 · **Faz:** 1.2 · **Durum:** Uygulandı

**Karar:** Encoder 400 kHz'de okunacak. 1 MHz denenmeyecek, kartın 10 kΩ
pull-up'ları değiştirilmeyecek.

**Ölçüm** (RAW_ANGLE okuma, 2 ms bütçeye göre):

| SCL | Süre | Pay |
|---|---|---|
| 100 kHz | 580 µs | %29 |
| **400 kHz** | **217 µs** | **%10.8** |
| 1 MHz | 157 µs | %7.9 |

**Gerekçe:** Süreler saat hızıyla orantılı düşmüyor — her okumanın
~130–150 µs'si ESP-IDF I2C sürücüsünün sabit yükü (semafor/görev geçişi),
bus süresi değil. 1 MHz sadece 60 µs (%3) kazandırıyor, karşılığında 10 kΩ
pull-up'ları Fast-mode Plus'ın istediğinin (4.7 kΩ) dışına zorluyor.
Faz 2.2'de bütçe sıkışırsa saldırılacak yer bus hızı değil, sürücü yükü
(asenkron API).

---

## K-012 — Pinpon topu ultrasonik için uygun hedef değil
**Tarih:** 2026-09-03 · **Faz:** 1.4 · **Durum:** Ölçüldü, sonuç K-013'e taşındı

**Gözlem:** Düz karton hedefle HC-SR04 kusursuz (`spread 0.04 cm`,
`missed 0`). Pinpon topuyla ise sensör topu **hiç görmedi**: beam
boyunca hareket ettirilirken 16 s boyunca 52.8 cm'de (beam'in ötesindeki
sabit yüzey) kilitli kaldı, ara sıra 210 cm'de (duvar) okudu.

**Fizik:** HC-SR04'ün demeti ~15° yarı açılı koni; 45 cm'de çapı ~24 cm.
4 cm'lik küresel top kesitin %3'ünden azını kaplıyor ve üzerine düşen sesi
her yöne saçıyor. Enerjinin kalanı arkadaki geniş düz yüzeye çarpıp çok
daha güçlü dönüyor; modül en güçlü yankıyı raporluyor.

**Geçici çözüm:** Düz yüzeyli bir araç beam'e takıldı; 5–41 cm aralığında
temiz ölçüm alındı. Ayrıca ~5 cm altında yankı vericinin çınlamasıyla
çakışıp değer şişiyor (3.50 cm gerçek → 4.06 okuma), ~2 cm altında hiç
echo yok. Bu bölge yazılımla düzeltilemez, fiziksel sınır.

**Tetikleme periyodu:** 2 ms'de `missed=0` ama yayılım 0.47 cm (önceki
atışın yankıları); 10 ms'de yayılım 0.04 cm; 30 ms uzak yansımaların
dürüstçe çözülmesi için gerekli (10 ms'de ölçüm tavanı ~163 cm, 180 cm
okumaları aliasing'di).

---

## K-013 — Konum sensörü olarak kamera (PS3 Eye), HC-SR04 yedek
**Tarih:** 2026-09-07 · **Faz:** 1.4 → C · **Durum:** Uygulandı

**Karar:** Top konumu PC'deki kameradan (PS3 Eye) hesaplanıp ESP32'ye
gönderilecek. HC-SR04 kablolu ve kodda kalır; yedek/karşılaştırma yolu.

**Gerekçe:** K-012. Araç çözümü çalışıyor ama gerçek pinpon topunu
kullanmak istendi; ultrasonik bunu fiziksel olarak yapamıyor. Kamera ölü
bölge bırakmıyor, beam boyunca mutlak konum veriyor, topla çalışıyor.

**Mimari karar — PC sensör, ESP32 kontrolcü:** PID, step üretimi ve
encoder okuma ESP32'de kalır; kamera sadece konum besler. Windows'un
gerçek zamanlı olmaması kontrol döngüsüne bulaşmaz, sensör
güncellemesini geciktirir. PC dururssa ESP32 bayatlık bekçisiyle güvenli
duruma geçer (K-017). Faz 2/3'ün geri kalanı bu kararla ayakta kalıyor.

**Dahili kamera reddedildi:** otomatik pozlama kilitlenemiyor, 30 fps,
p99 kare aralığı 49 ms.

---

## K-014 — PS3 Eye sürücüsü: libusbK + pseyepy (DirectShow yolu başarısız)
**Tarih:** 2026-09-07 · **Faz:** A/B · **Durum:** Uygulandı

**Denenen A yolu — PS3EyeDirectShow 1.0b2 (2019):** çözünürlük isteği yok
sayıldı (320×240 → 640×480 kaldı), pozlama/kazanç erişilemez (`-1.0`),
tam 1000 ms'lik takılmalar (p99 1000 ms, efektif 7 fps). Kaldırıldı.

**B yolu — Zadig ile Interface 0'a libusbK, Python'da `pseyepy`:**
kameranın yerel modlarına ve pozlama/kazanç register'larına doğrudan
erişim. `pseyepy` PyPI'da yok, GitHub'dan C uzantısı derlenerek kuruldu
(MSVC Build Tools mevcut). `pillow` ve `h5py` paket yüklenirken import
edildiği için zorunlu bağımlılık.

**Dikkat:** PS3 Eye'ın MI_01 (mikrofon) arayüzü Windows'un kendi sürücüsüyle
zaten çalışıyor; Zadig **yalnızca Interface 0**'a uygulanmalı.

---

## K-015 — Kamera çalışma noktası 320×240 @ 100 fps
**Tarih:** 2026-09-10 · **Faz:** B · **Durum:** Uygulandı

**Ölçüm** (1000 kare, pozlama sabit):

| fps | Efektif | Geç kare | maks aralık |
|---|---|---|---|
| 125 (exp 20/40/60) | 102–115 | %5–10 | 48–104 ms |
| **100** (exp 20/40/60) | **99.9** | **%0** | **11.9 ms** |

**Karar:** 100 fps. 125 fps'in kaybı pozlamadan bağımsız (USB izokron
sınırı); 8 ms yerine 10 ms almak karşılığında sıfır kare kaybı ve
deterministik dağılım (p99 10.5 ms). Kontrol döngüsü (2 ms) ile 5:1 oran.

**Yan bulgu:** 100 fps'te zamanlama pozlamadan tamamen bağımsız → pozlama
yalnızca görüntü kalitesine göre seçilebilir.

**Uyarı:** Kamera damgaları 1 ms çözünürlüklü; `cam interval`'daki 16–18 ms
"sıçramalar" yuvarlama artefaktı — duvar saatiyle birlikte okunmalı.

---

## K-016 — Her açılışta kalibrasyon sihirbazı, sabit eşik yok
**Tarih:** 2026-09-13 · **Faz:** C · **Durum:** Uygulandı (`gui/calibrate.py`)

**Sorun:** Gündüz tuned HSV eşikleri gece çalışmadı ve tersi. Sabit
pozlamada görüntü parlaklığı ortam ışığıyla birebir değişiyor; S ve V
eşikleri buna doğrudan bağlı. Ayrıca kamera/beam konumu oturumlar
arasında kayıyor.

**Karar:** Hiçbir eşik veya konum saklanmaz; `tracker.py` her açılışta
sihirbazı zorunlu çalıştırır (`--skip-calibration` yalnızca geliştirme,
uyarı basar). Sihirbaz:
1. **Pozlama normalizasyonu** — band parlaklığını hedefe (V≈110) getirip
   kilitler (kendi döngümüz; kameranın auto modu kullanılmaz)
2. **Beam ekseni** — iki tık: izdüşüm ekseni, ROI bandı, px→cm ölçeği
3. **Top rengi** — topa tıklayarak örnekleme; H dar, S/V sadece alt sınır
4. **Doğrulama** — top uçtan uca: tespit oranı ≥0.95, kapsama ≥0.80,
   alan oranı ≥0.40, band içinde kalma

**Doğrulama:** İki farklı ışıkta H aralığı 1 birim kaydı (9–25 → 10–26),
S/V 20–30 kaydı. H-ağırlıklı stratejinin gerekçesi bu.

**Sınır:** İki kalibrasyonda da pozlama 255'te (tavan), kazanç 44–56.
**DC LED aydınlatma önerildi** — şebeke lambaları 100 Hz titrer, 100 fps
ile örtüşür.

**Kamera yandan bakıyor:** konum = yatayda işaretlenen eksene izdüşüm;
eğim izdüşümü cos(θ) kadar kısaltır (±10°'de %1.5), kontrolcü taşır.

---

## K-017 — PC↔ESP32 bağlantısı: USB-Serial-JTAG, 100 Hz, bayatlık bekçisi
**Tarih:** 2026-09-13 · **Faz:** D · **Durum:** Uygulandı

**Taşıma:** ESP32-S3'ün yerleşik USB-Serial-JTAG portu (VID 303A:1001,
COM12). Konsol UART0/CH343'te (COM11) kalır;
`CONFIG_ESP_CONSOLE_SECONDARY_NONE=y` ile konsolun USB'ye yansıması
kapatıldı (log satırları paketlerin arasına girmesin). `platformio.ini`'de
`monitor_port`/`upload_port = COM11` sabitlendi.

**Paket** (7 bayt, kare başına bir, 100 Hz): `AA 55 | seq u8 | pos i16
LE 0.1 mm | flags u8 (bit0 valid, bit1 warning) | crc8`. CRC8-ATM,
TMC2208 ile aynı polinom → PC tarafında tek implementasyon. ESP32 her
paketi 4 baytlık echo ile yanıtlar (RTT ölçümü için).

**Bekçi:** 50 ms (5 kare) paket gelmezse `stale` — kontrol döngüsü bunu
görünce güvenli duruma geçecek.

**Ölçüm:** 1000+ pakette 0 CRC hatası, 0 seq boşluğu; RTT p50 1.14 ms,
p99 1.7 ms, maks 2.7 ms; ESP32'deki veri yaşı 0–7 ms. (İlk ölçümdeki
15.8 ms, PC okuyucusunun 20 ms zaman aşımıydı — ölçüm hatası.)

**Uçtan uca bütçe:** kare ≤10 ms + tespit 0.55 ms + bağlantı ~0.6 ms ≈
**11–13 ms**. HC-SR04'ün 30 ms'sine karşı ~2.5×.

---

## K-018 — AS5600 DIR pini harici olarak GND'ye bağlanmalı
**Tarih:** 2026-09-13 · **Faz:** 1.2 (yeniden test) · **Durum:** Uygulandı

**Olay:** Kablolama yenilendikten sonra encoder, mil sabitken her
örnekte iki değer arasında salındı: 3726↔370, sonra 3594↔502. I2C hatası
yoktu. 1 MHz hipotezi A/B ile çürüdü; kablo oynatma testi salınımı
yeniden üretti.

**Kök neden:** Her çiftin toplamı **4096** — aynı açının ters polariteyle
(`4096−x`) okunması. Bunu yalnızca **DIR** pini yapar. Kartın DIR'i
GND'ye bağladığı sanılıyordu (erken bir ölçüme dayanarak); değildi.
Boştaki DIR, I2C hattındaki aktiviteyle kapasitif kuplajla toggle oluyor.
Datasheet DIR için dahili pull belirtmiyor.

**Çözüm:** DIR'den GND'ye harici kablo. Sonrasında kablo oynatma testi:
309 örnek, 0 sıçrama, 0 tümleyen çift.

**Ders:** Encoder kablolaması artık **5 tel** (VCC, GND, SDA, SCL, DIR).
Faz 2 encoder okumasına makullük filtresi (2 ms'de fiziksel olarak
imkânsız sıçramayı reddet) ve `x + önceki == 4096` kontrolü eklenecek.
Kalıcı kurulumda kablolar lehimlenmeli.

---

## K-019 — Mutlak encoder: homing hareketi yok, saklanan ofset + makullük kontrolü
**Tarih:** 2026-09-13 · **Faz:** 3 hazırlık · **Durum:** Karar verildi

**Mekanizma:** Encoder motor milinde. Krank–biyel: mil → kısa kol → uzun
kol → beam. Beam yatay ⟺ krank yatay (9), biyel dikey. Çalışma aralığı
krank 6 → 9 → 12 (180°). Krank 6'da beam p1 tarafına, 12'de p2 tarafına
eğik. Driver kapalıyken beam ağırlığı krankı 6'ya düşürür; yatayda (9)
kaldıraç kolu maksimum → motorun en zor tutma noktası.

**Karar:** Açılışta homing hareketi yapılmayacak. AS5600 mutlak; mil
konumu tek okumayla biliniyor. Gereken tek şey **beam-yatay encoder
ofseti** — mıknatıs mile sabit olduğu için sürüklenmez, bir kez ölçülüp
saklanır (her açılışta yenilenmesi gereken ışık/kamera konumundan farklı
olarak rijit mekanik ilişki). Açılışta **makullük kontrolü**: okunan değer
180°'lik pencerenin dışındaysa çalışma reddedilir.

**Not:** Beam açısı krank açısının doğrusal olmayan fonksiyonu (≈sinüs);
yatay civarı yaklaşık doğrusal. Faz 3'te beam-açısı → krank-açısı
dönüşümünde hesaba katılacak.

---

## K-020 — Motor kablolama olayı ve doğrulanmış yön uzlaşımı
**Tarih:** 2026-09-14 · **Faz:** 3.3/3.4 hazırlık · **Durum:** Çözüldü

**Olay:** Beam bağlıyken ilk sürüş testinde motor komut edilenin **2–4 katı**
döndü ("vuruntu"), ters yönde ilk artışta stall etti. Sargı, yük ve
StealthChop hipotezleri sırayla incelendi.

**Kök neden:** Mekanizma kurulurken motor kablosu yanlış takılmıştı;
testler arasında düzeltildi. Kanıt: aynı `DIR=0` seviyesi iki
testte zıt encoder yönü verdi. Yüksüz test (biyel ayrı) düzeltilmiş
kablolamayla **bir komut turu = bir ölçülen tur** (+4116/−4096 sayım,
12.44 µadım/sayım, `ola/olb=0`, `CS=20`) gösterdi; yük altında ince tarama
(96 × 1.8°, iki yön) **sıfır stall, sıfır sıçrama, tur sonu +3 sayım**.
Yük ve StealthChop masum.

**Titreşim:** Testlerdeki dur-kalk hareket (ani başla/dur, rampasız) her
duruşta rotoru salındırıyordu; sürekli sabit hızda (3.75 rpm) tam tur
**pürüzsüz** — örnek başına artış 2.560 ± 0.5 sayım, sıçrama yok. Faz 2'de
step üretimi rampalı olmalı (K-009 ile uyumlu).

**Doğrulanmış yön uzlaşımı (düzeltilmiş kablolamayla; önceki elle
haritalama sonucu geçersiz):**

| DIR | Krank | Top |
|---|---|---|
| **1** | → 12 | → p2 (45 cm) |
| **0** | → 6 | → p1 (0 cm) |

Encoder 6 → 12 yönünde **azalıyor**. `IRUN=20` yatayda (maksimum kaldıraç)
yeterli.

---

## K-021 — Encoder hatası karakterize edildi, LUT ile düzeltilecek
**Tarih:** 2026-09-14 · **Faz:** 3 hazırlık · **Durum:** Uygulandı (`src/encoder_lut.h`)

**Ölçüm:** Yüksüz, sürekli, iki tam tur; her 32 µadımda (0.225°) okuma →
yön başına 1600 nokta. Komut konumu mikroadım sayısından kesin.

**Bulgu:** RAW_ANGLE tur boyunca **−2.8° … +6.0°** sapıyor. Harmonikler:
1. **3.05°** (mıknatıs eksen kaçıklığı — montajdaki kayma gözle de
teyit edildi), 2. **1.83°** (eğiklik), 3. 0.31°, 4. 0.05°. 4 harmonik model
artığı 0.043° rms. İleri/geri histerezis **0.17°** (rotor gecikmesi,
encoder hatası değil). Hata saf konum fonksiyonu, tekrarlanabilir.

**Çalışma yayında:** 6'dan 45°–110° arası hata düz (±1 sayım); yatay (≈87°)
bu düzlüğün ortasında, yerel ölçek 0.999. Yani çalışma noktası civarında
encoder artımsal olarak neredeyse kusursuz; mutlak ofset yatay
kalibrasyonunda yutulur. Hata 12'ye yaklaşırken −5.5°'ye büyüyor.

**Karar:** `src/encoder_lut.h` — 256 girişli (`raw>>4`) düzeltme tablosu +
doğrusal enterpolasyon, `encoder_correct(raw)`. İleri taramadan kuruldu,
geri taramada doğrulandı: düzeltmesiz maks 6.04° → **düzeltilmiş rms
0.049°, maks 0.22°**. Ham veri `docs/encoder_calibration_2026-09-14.csv`.

**Uyarı:** Tablo bu mıknatısın bu konumuna aittir. Mıknatıs yeniden
oturtulursa `test_motor_free` tekrar koşulup tablo yeniden üretilmeli.
Mıknatısı yeniden merkezlemek 1. harmoniği düşürür — isteğe bağlı.

**Faz 2 için:** encoder modülü = `encoder_correct()` + K-018 makullük
filtresi (`x+önceki≈4096` tümleyen kontrolü, 2 ms'de imkânsız sıçrama
reddi).

---

## K-022 — Kontrol döngüsü 4 ms; GPTimer jitter doğrulandı; döngüde log yok
**Tarih:** 2026-09-14 · **Faz:** 1.1 · **Durum:** Doğrulandı, Faz 1 kapandı

**Karar 1 — periyot 2 ms → 4 ms (250 Hz).** Her çevrim başına maliyet
göreli olarak yarıya iner (encoder 222 µs → %5.5); kamera 100 Hz konum
akışı çevrim başına 2.5 güncelleme alır; ball-beam için 250 Hz fazlasıyla
yeterli. `board_pins.h`: `CONTROL_LOOP_PERIOD_US 4000`.

**Ölçüm — yazılım (`test_timer_jitter`, 15 s, 3745 örnek):** GPTimer
ISR'ları arası 4000.00 µs, min=max=4000, std 0.00 (GPTimer ve `esp_timer`
aynı kristalden). ISR→görev uyanma **11 µs, sabit**. Kaçırılan bildirim 0.

**Ölçüm — donanım (mantık analizörü, GPIO16, 4 MHz örnekleme, 1167
periyot):** ortalama 3999.87 µs, **|maks sapma| 0.29 µs**, std 0.157 µs.
Kabul ±50 µs → **172× marj.** Uzun dönem −32.7 ppm (kristal toleransı,
önemsiz). Dışa aktarımın ilk ve son satırları kayıt başlangıç/bitiş
damgası — kenar değil, analizden çıkarılmalı.

**Karar 2 — kontrol görevinde konsol çıkışı yasak.** Test sırasında
görevin bastığı tek bir ek `printf` satırı, o çevrimde ISR→görev
gecikmesini 11 µs'den **2841 µs**'ye çıkardı. Telemetri kuyruğa yazılır,
düşük öncelikli ayrı görev basar. `ESP_LOG`/`printf` kontrol yolunda
kullanılmaz.

**Mimari sonuç:** Kontrol döngüsü ISR'da değil, ISR'ın uyandırdığı
**Core0 yüksek öncelikli görevde** koşabilir — 11 µs sabit uyanma
gecikmesi bunu güvenle mümkün kılıyor. ISR yalnızca zaman damgası +
bildirim (plandaki "ISR'de iş yok" kuralı korunuyor).

---

## K-023 — Faz 2 iskeleti: LEDC+PCNT step üretimi, korkuluklar, ölçülmüş bütçe
**Tarih:** 2026-09-14 · **Faz:** 2.1–2.4 · **Durum:** Uygulandı ve tezgâhta doğrulandı (`d986da6`, `feature/control-core`)

**Görev şeması:** Core 0: GPTimer 4 ms ISR (zaman damgası + GPIO16 probe +
bildirim) → `control` görevi (prio 23, log yok). Core 1: `link` (2 ms
`pc_link_poll`), `diag` (100 ms TMC bekçisi), `telem` (kuyruk → konsol,
10 Hz `S,` satırı + 1 Hz `B,` bütçe satırı). Faz 1 testleri `#ifdef
BB_TESTS` ile korunuyor; `pio run -e tests` ile derlenir
(`build_src_filter` ESP-IDF'te çalışmıyor).

**Karar 1 — step üretimi donanımda:** STEP = LEDC kare dalga (10 bit,
APB 80 MHz → 76–78 000 Hz; 80 µadım/s altı "durdu", darbe yok). Frekans =
hız, her tikte bir kez yazılır. Aynı STEP pini **PCNT**'ye geri okunur,
DIR seviye girişi olarak yön verir → komut edilen konum *gerçek darbe
sayısı*, frekans integrali değil. Hız rampası (`STEP_AMAX` 8000 µadım/s²)
`stepper_tick` içinde; DIR yalnızca durma bandında değişir.

**Karar 2 — korkuluklar (kilitli FAULT: darbe kes + driver kapat):**
encoder [6−100, 12+100] dışı; komut−ölçüm >150 sayım; 25 tik ardışık kötü
okuma; `GSTAT.reset` (yeniden yapılandırma değil — koşan profil altında
1/8'e düşmüş sürücüye güvenilmez, operatör yeniden başlatır); UART sessiz.
Link bayat (>50 ms) → **HOLD**: rampayla dur, konumu tut (IHOLD), link
dönünce devam. Açılışta beam 6'da ±150 sayım değilse çalışma reddi (K-019).

**Karar 3 — takip hatası göreli:** Krank "6"da serbest sallanır, dinlenme
noktası ±40 sayım oynar (535 kayıtlı; bugün 567/576). Darbe sayacı motor
açılırken sıfırlandığı için `follow_err` = komut − (encoder − açılış
encoder'ı). Mutlak konum (K-019 ofseti) log ve aralık korkuluğunda aynen.

**Ölçüm — Faz 2.2 bütçe (4000 µs periyot, 150+ s):**

| Faz | Ort | Maks |
|---|---|---|
| Encoder (I2C 400k + LUT + filtre) | 231 µs | 361 µs |
| Hesap | 9 µs | 46 µs |
| Step (rampa + `ledc_set_freq`) | 2 µs | 105 µs |
| **Toplam** | **244 µs (%6)** | **467 µs (%11.7)** |

ISR→görev 10 µs; aşım 0; kaçırılan tik 0; kuyruk düşen 0.

**Ölçüm — açık döngü hareket:** üçgen 6 → +1000 sayım → 6, 3200 µadım/s
(3.75 rpm), 1 s bekleme. Komut 1000.6 sayım → encoder 1000.9;
`follow_err` **0.4–4.9 sayım** (LUT + motor tutarlı). Her dönüşte aynı raw
(576). Link 100 Hz: 0 CRC, 0 kayıp. Encoder 0 I2C hatası / 0 ret / 0 DIR
arızası. TMC: reset 0, hareketle CS 12→20, `stst` doğru.

**FAULT testi (encoder kablosu çekildi):** `ENC_DEAD` 100 ms'de kilitlendi,
driver kapandı. Ama ilk denemede ESP-IDF I2C sürücüsünün her başarısız
işlemde *çağıran görevden* bastığı iki `ESP_LOGE` satırı döngüyü 14.5 ms'ye
çıkardı (250/250 aşım) — K-022 ihlali sürücüden geldi. Düzeltme:
`esp_log_level_set("i2c.master", ESP_LOG_NONE)` + zaman aşımı 2 → 1 ms.
Tekrar: encoder fazı 1417 µs ort, bütçe %46, 0 aşım, konsol temiz; kablo
takılınca okumalar reset'siz toparlıyor, FAULT kilitli kalıyor.
**Kural:** kontrol yolundan çağrılan her sürücünün log etiketi susturulmalı
(ileride: `ledc`, `pcnt`, `gptimer`).

**Not:** Konsol tamponu (CH343) port kapalıyken ~8 s veri biriktiriyor;
canlı okuma yapan araç önce `reset_input_buffer` + 1 s boşaltma yapmalı.

---

## Faz 1 kapanış özeti (2026-09-14)

| Alt adım | Sonuç |
|---|---|
| 1.1 Timer | 4 ms, jitter 0.29 µs (donanım), ISR→görev 11 µs |
| 1.2 AS5600 | 400 kHz, 222 µs; DIR harici GND (K-018); LUT ile 0.05° rms (K-021) |
| 1.3 TMC2208 | UART doğrulamalı yapılandırma, 1/256, IRUN=20; yön uzlaşımı K-020 |
| 1.4 Mesafe | HC-SR04 yedek; **asıl sensör kamera** (K-013), 100 fps, PC bağlantısı D fazı |

---

## Faz 2 kapanış özeti (2026-09-15)

Kapandı; `feature/control-core` → `main` (merge), Faz 3
için `feature/pid-core` açıldı.

| Alt faz | Sonuç |
|---|---|
| 2.1 Görev/öncelik mimarisi | Core 0: ISR + kontrol görevi; Core 1: link, diag, telemetri (K-023) |
| 2.2 Bütçe ölçümü | %6 ort / %11.7 maks; encoder arızasında %46, aşım yok |
| 2.3 Ölçüm zaman damgası | `pc_link` `received_at_us` + bayatlık → HOLD; kamera gecikmesi PID'de hesaba katılacak |
| 2.4 Step üretici | LEDC + PCNT, rampalı; sabit hız yerine ivme sınırlı hız (K-009 gereği) |

Doğrulanmış güvenli durumlar: HOLD (link bayat), FAULT `ENC_DEAD`.
Kodda ama tetiklenmemiş: `ENC_RANGE`, `FOLLOW_ERR`, `TMC_RESET`, `TMC_UART`.

Faz 3'e devredilen: beam-açısı ↔ krank-açısı geometrisi (K-019 notu),
yatay ofset sabitinin ölçümü, `ledc`/`pcnt`/`gptimer` log etiketlerinin
susturulması, VMAX'ın kapalı çevrim ihtiyacına göre yükseltilmesi.

---

## K-024 — Faz 3 geometri, PID yapısı ve tezgâh ölçümleri
**Tarih:** 2026-09-20 · **Faz:** 3.1 / 3.3 / 3.4 · **Durum:** Ölçüldü (`a513239`, `19597f2`)

**Geometri (referans STL'lerden + ölçü):** krank r = 31 mm (Crank.STL),
biyel l = 90 mm (Coupler.STL, 605ZZ yuvaları), motor mili 60 / mafsal
150 mm yükseklik → biyel dikeyken beam yatay; **d = 477 mm** (mafsal pimi
↔ biyel pimi, düzenekte ölçüldü); krank 9'da pim mafsal kulesinden uzağa.
`linkage.c`: tam kinematik (φ→θ Newton, θ→φ daire kesişimi).

**Bulgu — eğri asimetrik ve tepe noktalı:** biyel kısa olduğu için pimin
yatay kayması biyeli eğiyor: θ(−90°) = −4.33°, θ(+90°) = +3.09°, tepe
**+3.26° @ φ=+75°**, sonrasında kazanç tersine dönüyor. Karar: `θmax = 3.0°`
(krank ≈ +57°/−52°). Bu bulgu simülasyonda "ulaşılamaz θ" hatası olarak
kendini gösterdi (θmax 3.3 ile krank hiç kıpırdamadı).

**Karar — PID yapısı:** PID çıkışı **beam açısı** (θ, ±3°); linkage ile
krank sayımına çevrilir; step motor konum-servo (`stepper_track`,
sqrt-profil), encoder korkuluk. Türev ölçüm üzerinde, 1. derece filtreli
(τ = 60 ms); anti-windup koşullu integral + kelepçe. Tesis modeli
x'' = (5/7)g·sinθ ≈ 700·θ cm/s²; ωn = 3 rad/s, ζ = 0.7 → **Kp 0.74 °/cm,
Kd 0.34 °·s/cm, Ki 0** başlangıç.

**Simülasyon (`test_pid_sim`, ESP üzerinde, 100 Hz kamera + 20 ms gecikme
+ 0.3 mm gürültü):** 7.5 cm basamak → yükselme 0.54 s, aşım %0.9, oturma
0.97 s @ 43 000 µadım/s; **3200 µadım/s'de kararsız** (hız doyması faz
kaydırıyor). Ki = 0.3: aşım %9.5 → şimdilik Ki yok. Anti-windup: doymuş
başlangıçta Ki'li/Ki'siz aşım aynı (%22, fiziksel sınır). Not: gürültü Kd
üzerinden θ komutunu ±0.15° titretiyor (krank ±2°) — donanımda izlenecek.

**Tezgâh (`test_jog`):** **`ENC_LEVEL_COUNTS = 1101`** (raw 3545, 96.8°
6'dan; su terazisi). Hız taraması ±600 sayım, 3200 → 43 000 µadım/s:
**adım kaybı yok**, hareketteki maks takip hatası 20 sayım (1.7°),
duruşta +4…+9 sayım her hızda aynı ve hep pozitif → gravite altında rotor
alanın gerisinde (alt konumda daha büyük). `STEP_VMAX_CLOSED_LOOP 43000`,
`AMAX 430000` doğrulandı. Telefon eğimölçer (1° çözünürlük) ±1.66°'lik
model açılarını ayırt edemedi — geometri doğrulaması kapalı çevrime kaldı.

---

## K-025 — Kapalı çevrim tezgâhta: ilk koşu, titreşim analizi ve ayarlar
**Tarih:** 2026-09-20 · **Faz:** 3.2 / 3.5 / 3.6 · **Durum:** Çalışıyor (`686edaa`, `3f4519f`; gui `9869d40`)

**Yapı:** WAIT → ENGAGE (driver 6 civarında açılır, 200 ms sonra mil
referansı — rotor enerjilenince kutba sıçrıyor, 30–40 sayım) → LEVEL_HOLD
(top yok / link bayat → yatay) ⇄ RUN (kamera karesi başına PID → θ →
linkage → krank hedefi). Açılış penceresi ±400 sayım (sürtünme krankı 6'dan
20°'ye kadar uzakta bırakıyor). Encoder telafisi: krank dururken
(hedef − ölçülen) integrali, τ 1.5 s, ±60 — yük açısını (statik 4–9,
açılışta 40 sayım) yutuyor; 0.3 s ile top döngüsüyle aynı banda giriyordu.

**İlk koşu:** yön doğru (sihirbazda p1 = motor ucu, p2 = mafsal ucu);
top hedefte, 1 cm sabit hata (Ki = 0) → Ki 0.15. Motor/sürücü sıcak
(IRUN 20 = 1.16 A rms; `vsense=0` → CS 31 = 1.77 A).

**Titreşim analizi (50 Hz ESP + 100 Hz kamera kayıtları):**
- Top dururken (22.5 ± 0.05 cm) krank ±1.8° / 20 ms geziniyordu: kamera
  gürültüsü (0.02 cm) × Kd → D ±0.2°. `PID_D_TAU_S` 0.06 → 0.15: yarıya
  indi (0.064° θ std). Sistemin kendi salınımı **yok** (60 s, spektrum DC).
- Ölü bant 14 sayım: 1.2°'lik adımlı sınır çevrimi, top ±0.5 cm → reddedildi;
  5 sayım kaldı.
- Asıl ses kaynağı hareketin biçimi: sqrt-profil 14 sayımlık düzeltmeyi
  12 000 µadım/s ve 430 000 µadım/s² ile "fırlatıyordu". `stepper_track`'e
  doğrusal bölge (`v = min(vmax, √(2ae), 40·e)`) → küçük düzeltmeler 1–5 rpm
  kayma; büyük hatalarda sqrt/VMAX aynen. Simülasyon: bozucu toparlama
  1.8 → 3.1 s (yalnızca son 3 mm); tezgâhta bu seviye yeterli bulundu.
- Krank hız/ivme sınırını düşürmek (430k → 150k) simülasyonda toparlamayı
  belirgin bozdu (5 s) → 43 000 / 430 000 kaldı.
- Kaydedilen "kendiliğinden" sıçramalar (durgun → 15 cm/s tek karede)
  fizik gereği dış darbe (itme/masa); kontrolör o anlarda sakin. Toparlama
  1.2 s, 0.7 cm aşım.
- Denge yatayın ±1°'si içinde herhangi bir yerde oturuyor (I −0.6…+0.95°):
  içi boş topun rayda ölü bandı. Not: tesis katsayısı (5/7)g değil (3/5)g.

**Akım:** IRUN 10 (0.61 A rms), IHOLD 4: takip hatası maks 8 sayım, adım
kaybı yok; `otpw` hiç yanmadı. Kalıcı çözüm Faz 6: durgunda IHOLD'a düşme +
sürücüye soğutucu.

**Araç:** `gui/link.py --log` kare-kare CSV; `--skip-calibration` yalnızca
tuning oturumunda (ışık/kamera değişmedikçe). ESP reset'i USB-JTAG portunu
düşürüyor → link.py kapanıyor (Faz 4.4 yeniden bağlanma).

**Bütçe:** %16 (PID + linkage 40 µs), aşım 0, link 0 CRC.

---

## Faz 3 kapanış özeti (2026-09-20)

Kapandı; `feature/pid-core` → `main` (`f4698e1`), Faz 4
için `feature/protocol` açıldı.

| Alt faz | Sonuç |
|---|---|
| 3.1 PID izole test | `test_pid_sim` (ESP üzerinde simüle tesis): basamak 0.97 s, aşım %0.9, anti-windup doğrulandı |
| 3.2 Kapalı çevrim | Kp 0.74 / Ki 0.15 / Kd 0.34, D τ 0.15 s; top 22.47 ± 0.03 cm |
| 3.3 Mekanik dönüşüm | Tam kinematik (K-024), yatay ofset 1101 sayım |
| 3.4 Encoder vs komut | Adım kaybı yok; yük açısı encoder telafisiyle; korkuluk 150 sayım |
| 3.5 Bozucu reddi | 15 cm/s itme → 1.2 s, 0.7 cm aşım |
| 3.6 Zaman damgalı log | `S,` satırı (50 Hz) + `gui/link.py --log` |

Faz 6'ya devredilen: sürücü ısısı (soğutucu, durgunda IHOLD), içi boş
topun ±1° ölü bandı, kazanç son ayarı, telemetri decimation 25'e dönüş.
Faz 4'e devredilen: ESP reset'inde USB-JTAG portunun düşmesi → otomatik
yeniden bağlanma (4.4).

---

## K-026 — Faz 4: USB binary protokol, komutlar, yeniden bağlanma; iki kök neden
**Tarih:** 2026-09-20 · **Faz:** 4.1–4.5 · **Durum:** Çalışıyor (`1c9e57c`; gui `497d44a`)

**Protokol (`protocol.h` ↔ `protocol.py`, alan alan aynı):** `AA 55 | tip | uzunluk |
yük ≤64 | crc8`. ESP→PC: `TELEM` 125 Hz (31 B: x, setpoint, P/I/D/θ 0.01°,
φ 0.1°, encoder, takip hatası, telafi, durum, arıza, bayraklar, son seq,
döngü µs, hız), `HEALTH` 1 Hz (bütçe, link/encoder/TMC sayaçları, komut ve
düşen çerçeve sayaçları, DRV_STATUS), `ACK` (komut tipi + sonuç + nonce),
`CONFIG` (kazançlar, limitler, `defaults` bayrağı — 5.5 için). PC→ESP: `POS`,
`SETPOINT` (3–42 cm), `GAINS` (Kp≤3, Ki≤1, Kd≤2, τ≤1), `MODE`
(RUN/LEVEL/STOP/RESET_FAULT), `PING`, `GET_CONFIG`. Link görevi doğrular ve
ACK'ler, kontrol görevi kuyruktan uygular. Konsol satırları artık isteğe bağlı.

**PC (`esp_link.py`):** VID/PID ile bul, PING→CONFIG el sıkışma, her G/Ç
hatasında kapat + 0.5 s'de bir yeniden tara; ESP reset'inden **0.98 s** sonra
bağlı. Windows zamanlayıcı 1 ms (`timeBeginPeriod`), okuma 1 bayt + `in_waiting`
(20 ms okuma penceresi RTT'yi 16 ms gösteriyordu). Ölçüm (`link_test.py`):
**RTT p50 2.1 ms**, TELEM periyodu p50 7.7 / p99 9.9 ms, 0 CRC, tüm ACK'ler doğru.

**Kök neden 1 — "USB ölü":** `main`, beam 6'da değilse `pc_link_init`'ten önce
duruyordu; ROM CDC'si enumerate olup cevap vermiyordu (yazma zaman aşımı, 0
bayt). Bugün krank sürtünmeyle 44°'de kaldığı için bütün oturum boyunca böyle
görünüyordu. Karar: **link her şeyden önce** başlar; başlangıç penceresi/init
hataları `FAULT_NOT_AT_REST` / `ENC_DEAD` / `TMC_UART` olarak telemetriye
düşer ve `RESET_FAULT` ile kurtarılır. Kural: kart hangi durumda olursa olsun
PC'den görünmeli.

**Kök neden 2 — kontrol görevi asıldı:** okumanın ortasına gelen chip reset
AS5600'ü SDA'yı düşük tutarak bıraktı; IDF I2C sürücüsü NACK sonrası
`while (bus_busy)` ile **zaman aşımsız** dönüyor → prio 23 görev sonsuz
döngü, IDLE0 açlığı, task watchdog (USB'den okundu, backtrace
`i2c_ll_is_bus_busy ← encoder_update`). Motor enerjili, döngü ölü. Kararlar:
`encoder_init` başında **koşulsuz 9 saat darbesi + STOP** kurtarma; 6/6
ardışık reset testi temiz. `CONFIG_ESP_TASK_WDT_PANIC=y`: asılan döngü →
yeniden başlat → driver kapalı (güvenli) → link geri, HEALTH'te görünür.

**Not:** USB-JTAG'de RTS asserted ile port açmak chip'i resetler
(`rst:0x15 USB_UART_CHIP_RESET`); pyserial'de DTR/RTS açılıştan önce düşük
tutulmalı (mevcut). COM11 RTS darbesi (EN) test için reset aracı.

---

## Faz 4 kapanış özeti (2026-09-20)

Kapandı; `feature/protocol` → `main`, Faz 5 için
`feature/gui` açıldı (gui repo: `main`, `497d44a`).

| Alt faz | Sonuç |
|---|---|
| 4.1 Çerçeve | `AA 55 · tip · len · yük ≤64 · crc8`, `protocol.h` ↔ `protocol.py` |
| 4.2 ESP→PC | TELEM 125 Hz, HEALTH 1 Hz, ACK, CONFIG |
| 4.3 PC→ESP | POS, SETPOINT, GAINS, MODE, PING, GET_CONFIG — aralık kontrolü + ACK |
| 4.4 Otomasyon | VID/PID, el sıkışma, reset sonrası 0.98 s yeniden bağlanma |
| 4.5 Loopback | RTT p50 2.1 ms, TELEM 7.7 ms p50, 0 CRC; kamera ile uçtan uca: top canlı setpoint 25.0'da ±0.03 cm |

Faz 5'e devredilen: kamera modunda RTT 6–9 ms görünmesi (pseyepy GIL →
okuyucu iş parçacığı gecikmesi; iş parçacığı ayrımı), `--skip-calibration`
yalnızca geliştirici kısayolu (varsayılan sihirbaz). Faz 6: sürücü ısısı,
durgunda IHOLD.

---

## Faz 5 kapanış özeti (2026-09-22)

Kapandı; firmware `feature/gui` → `main` (`b2bfdb3`),
Faz 6 için `feature/autotune` açıldı. GUI repo: `main` (`618daad`).

| Alt faz | Sonuç |
|---|---|
| 5.1 Bağlantı + arka plan okuma | Kamera ve link **ayrı süreçte** (`link_process.py`); pseyepy kare okurken GIL'i tuttuğu için iş parçacığı yetmedi (arayüz donuyor, konum 50 ms'i aşıp LEVEL'a düşüyordu). GUI kuyruklardan 25 Hz okuyor. |
| 5.2 Görselleştirme | Şema: p2 mafsal sabit, p1 motor ucu iner-kalkar, krank göstergesi; eğim kutuya ölçekli (taşma yok), çizim filtreli (gürültü titreşimi yok) |
| 5.3 Canlı grafikler | pyqtgraph, 20 s pencere, X ekseni kilitli, otomatik kaydırma + "Grafikleri ortala" |
| 5.4 Setpoint | Slider + spinbox + şemaya tıklama → `SETPOINT` |
| 5.5 Kazançlar | CONFIG'den dolar, "varsayılan (ESP)" etiketi, Uygula/Varsayılana dön (CONFIG derlenmiş varsayılanları da taşıyor) |
| 5.6 Durum/sağlık | Sekmeli panel (Genel/Encoder/Motor/Link/Kamera), kutu başına bir değer, DRV_STATUS çözümlü; **uyarı şeridi**: arıza kırmızı, ön-belirti turuncu (encoder I2C/ret/DIR, TMC OTPW/OT/kısa devre/açık faz/UART, takip hatası, döngü aşımı) |
| 5.7 Kayıt | CSV: her TELEM karesi + gönderilen her konum (Faz 6 analizleri için) |

Ek: cihaz yokluğu yönetimi — ESP ve kamera için ayrı şerit, kamera yoksa
süreç ölmüyor (link canlı, 3 s'de bir ve "Yeniden tara" ile deniyor);
açılışta kayıtlı kalibrasyon varsa "eski ayarla devam / yeni kalibrasyon"
sorusu (K-016'nın bilerek gevşetilmesi).

**Bulunan firmware hataları:** (1) `MODE RUN` STOP/FAULT'tan yeniden
devreye almıyordu; (2) yeniden devreye girişte darbe sayacı eski
oturumdan devam edip anında `FOLLOW_ERR` (900 sayım) atıyordu — sayaç,
hedefler ve telafi artık her devreye girişte sıfırlanıyor. Tezgâhta tüm
geçişler doğrulandı.

---

## K-027 — Tesis ölçüldü; otomatik ayar reddedildi, elle bulunan kazançlar doğrulandı
**Tarih:** 2026-09-23 · **Faz:** 6 · **Durum:** Karar verildi — otomatik ayar devre dışı

**Yeni firmware yeteneği (kalıcı):** `THETA` komutu + `MODE_OPENLOOP`
durumu — beam açısı PID yerine doğrudan PC'den. ±2° sınırlı, **500 ms
komut gelmezse kendi kendine yatay**, link bayatsa yine yatay; tüm
korkuluklar geçerli. Tezgâhta doğrulandı (0.5→1.5° komutlar birebir,
krank 7.7→23.8°, takip hatası ≤3 sayım, zaman aşımı beam'i indirdi,
5° reddedildi).

**Ölçüm yöntemi (`gui/autotune.py`):** röle/Ziegler-Nichols değil —
çift integratör için tanımsız ve 45 cm'lik beam'de topu uçurur. Bunun
yerine sınırlı açık-döngü deneyleri: gürültü (yataydan, **ikinci derece
uydurma** ile topun kendi hareketi ayıklanır), ölü bant, ve **simetrik
±1° darbe çiftleri** — a = K(θ−θ₀) modelinden
`K = (a₊ − a₋)/2θ`, `θ₀ = −(a₊ + a₋)/2K`; böylece rayın hafif eğikliği
kazancı yanıltmaz, üstelik büyüklüğü de çıkar. Gecikme kameradan değil
**krank telemetrisinden** (temiz sinyal) + sabit kamera gecikmesi.
Her darbe PID ile ortalanmış topla başlar; top ±9 cm dışına çıkarsa,
link/top kaybolursa veya arıza olursa deney durur.

**Ölçülen tesis:** K = **605 cm/s²/rad** (teorik 600 — içi boş top
varsayımı doğrulandı), gecikme **45 ms**, kamera gürültüsü 0.28 mm,
ölü bant 0.15°, yatay ofset ölçümü 0.1–0.4° (darbe öncesi tam durgunluk
sağlanamadığı için üst sınır).

**Doğrulama (22.5 → 27.5 cm basamağı):**

| | Aşım | Oturma | Kalıcı hata | θ titreşimi |
|---|---|---|---|---|
| Elle (Kp 0.74 / Ki 0.15 / Kd 0.34 / τ 0.15) | %3 | **1.04 s** | −0.18 cm | **0.054°** |
| Kutup yerleştirme (0.85 / 0.09 / 0.43 / 0.25) | %7 | 2.00 s | −0.19 cm | 0.131° |

Öneri daha kötü çıktı, tuner **kendi önerisini geri aldı** (tasarlandığı
gibi). **Neden:** Kd büyüdükçe türev filtresi (τ_D) de büyümek zorunda
(gürültü bütçesi), kazanılan hız faz kaybıyla geri veriliyor. Yani
sistemin sınırı tesis kazancı değil, **kamera gürültüsü / türev
filtresi**. Elle bulunan takım gerçekten iyi bir noktada.

**Karar:** GUI'ye otomatik ayar düğmesi eklenmedi; `autotune.py` mekanik
değişiklik sonrası tesisi yeniden ölçmek için tezgâh aracı olarak kalıyor
(`tools_run_autotune.py` ile çalışır). Kazançlar elle bulunan değerlerde.

**Yol boyunca bulunan firmware hataları:** telemetri `x`'i yalnızca RUN'da
güncelleniyordu (OPENLOOP/LEVEL/STOP'ta donuyordu — açık döngü deneyleri
kördü); `MODE_OPENLOOP` protokol doğrulamasından geçmiyordu.

---

## K-028 — Son PID kazançları: aşımın kaynağı sönüm, çözüm Kd 0.34 → 0.50
**Tarih:** 2026-09-24 · **Faz:** 6 · **Durum:** Uygulandı (firmware varsayılanı)

**Belirti:** Sistem hedefi geçiyor, salınarak sonradan oturuyor.

**Ölçüm:** `gain_sweep.py` ile aday kazanç setleri tezgâhta ±5 cm adımlarla
dört bağımsız taramada puanlandı; ardından `ab_test.py` ile eski/yeni set
kapalı çevrimden hiç çıkmadan karşılaştırıldı (tarama aracı adımlar arasında
0,3 s açık çevrime düşüp topu kaçırıyordu — A/B aracı düşmüyor).

**Kök neden:** Sönüm oranı. K_deg = 10,56 cm/s² (derece başına) ile
`ζ = Kd·K_deg / (2·√(Kp·K_deg))`; Kd 0,34 → **ζ = 0,64** (belirgin aşım),
Kd 0,50 → **ζ = 0,94** (kritik sönüme yakın).

**Karar:**

| | Kp | Ki | Kd | τ_D |
|---|---|---|---|---|
| eski | 0,74 | 0,15 | 0,34 | 0,15 |
| **yeni (varsayılan)** | **0,74** | **0,06** | **0,50** | **0,15** |

**Sonuçlar** (±5 cm adım, aynı gün, aynı düzenek):

| | ortalama aşım | oturma | kalıcı hata |
|---|---|---|---|
| eski | %17,3 (tek adımda %26'ya kadar) | — | — |
| yeni | **%1,9** | 1,5 s | +0,14 cm |

Altı adımlık tekrarlanabilirlik koşusunda yeni setin adım adım dağılımı
%−8,7 … %+16,4. **Bu dağılım kontrolcüden değil:** topun ray üzerindeki ölü
bandı/tutunması 0,15–0,3°, yani ±0,2–0,5 cm konum belirsizliği demek.
Merkezden uzaklaşan adımlar bu yüzden dar bir banda hiç oturmuyor.

**Neden daha fazlası değil:**
* Kp > ~0,85 45 ms çevrim gecikmesine tosluyor (Kp 0,95 → %17,8 aşım).
* Ki 0 ile 0,06 arası fark gürültünün içinde kalıyor; Ki yalnız ölü bandı
  sürünerek geçmeye yarıyor, büyütmek ölçülebilir bir şey kazandırmıyor.
* Kd 0,50'nin ötesi motor vızıltısını ve mekanik gürültüyü artırıyor.
* Adım aşımını tamamen sıfırlayacak yol **hedef rampası** (iç setpoint'i
  ~4 cm/s ile süzmek) veya β ağırlıklı setpoint; mevcut başarım yeterli
  görüldüğü için (2026-09-26) eklenmedi.

**Ölçüm tuzağı (tekrar edilecekse):** oturma bandı 0,3 cm alınırsa adımların
yarısı "oturmadı" sayılır ve ortalamalar `nan` ile bozulur. Tesisin kendi
kalıcı hatası 0,2–0,5 cm olduğu için band **≥ 0,5 cm** olmalı
(`autotune.STEP_SETTLE_BAND_CM = 0.7` bu yüzden seçilmişti).

---

## Referans: Doğrulanmış pin haritası (Faz 1.3 sonu)

| Sinyal | ESP32-S3 | Notlar |
|---|---|---|
| STEP | GPIO4 | |
| DIR | GPIO5 | |
| UART TX | GPIO6 | 1kΩ seri direnç üzerinden PDN pinine |
| UART RX | GPIO7 | doğrudan PDN pinine, dirençsiz |
| EN | GPIO15 | LOW = aktif |
| CLK | — | sürücüde GND'ye bağlı (dahili 12MHz osilatör) |
| VIO | 3V3 | |
| GND | GND | ESP32 / sürücü lojik / sürücü motor / PSU ortak |

MS1, MS2 boşta (UART'tan `mstep_reg_select=1` ile devre dışı).
VM = 12V, motor sargıları M1A/M1B ve M2A/M2B.

### AS5600 encoder (Faz 1.2, K-018 sonrası)

| AS5600 | ESP32-S3 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO8 |
| SCL | GPIO9 |
| **DIR** | **GND — harici kablo, zorunlu** (K-018) |

OUT, GPO boşta. Kart üzerinde 10 kΩ pull-up var; ESP32 dahili pull-up'lar
kapalı. 400 kHz.

### HC-SR04 (Faz 1.4, yedek)

TRIG GPIO10, ECHO GPIO11 (1k/2k bölücü üzerinden, 5V→3.3V), VCC 5V
(PCB'de köprülenen pad, ~4.7V), GND ortak.

### PC bağlantısı (Faz D)

USB-Serial-JTAG (native USB port, COM12) ↔ `gui/link.py`. Konsol
UART/CH343 (COM11). Karta iki USB kablosu takılı.

## Referans: Faz 1.3'te yazılan register değerleri

| Register | Değer | Anlamı |
|---|---|---|
| `GCONF` (0x00) | `0x000000C0` | `pdn_disable=1`, `mstep_reg_select=1`, `i_scale_analog=0` |
| `CHOPCONF` (0x6C) | `0x100101B5` | `MRES=0` (1/256), `intpol=1`, `TBL=2`, `HSTRT=4`, `HEND=0`, `TOFF=5` |
| `IHOLD_IRUN` (0x10) | `0x00041008` | `IHOLD=8`, `IRUN=16`, `IHOLDDELAY=4` |

UART: 115200 baud, 8N1, tek hat half-duplex, CRC8-ATM.
Doğrulanan okumalar: `VERSION=0x20`, `SEL_A=1`, `uv_cp=0`, `ENN=0`,
hareket sırasında `stst=0`, `CS_ACTUAL=16`.

**Uyarı:** `IRUN=16` bir bring-up değeri, motorun nominal akımına göre
ayarlanmadı. Kartın sense direnci ölçülüp datasheet bölüm 9'daki
`I_RMS = ((CS+1)/32) × (V_FS / (R_SENSE + 30mΩ)) × (1/√2)` formülüyle
hesaplanmalı (Faz 3 öncesi).
