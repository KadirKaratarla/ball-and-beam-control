# 3D parçaların kaynağı ve kullanım koşulları

Bu klasördeki STL dosyaları ve `BALL AND BEAM ASSEMBLY.zip` (SolidWorks
kaynak parçaları) **bu projenin eseri değildir.** Kaynak:

> **IVProjects / Engineering_Projects** —
> <https://github.com/IVProjects/Engineering_Projects/tree/main/ProjectFiles/Ball%20and%20Beam%20Control%20System>
> Alındığı tarih: 2026-09-26

Düzeneğin mekanik parçaları bu modellerden basıldı; buradaki kopyalar
ölçüleri doğrulamak ve parçayı yeniden basmak için referans olarak
tutuluyor.

## Lisans durumu — dikkat

Kaynak deponun **LICENSE dosyası yok.** Lisans belirtilmemiş bir eser
varsayılan olarak "tüm hakları saklıdır" sayılır: dosyaları kendi açık
depomuzda **yeniden dağıtma hakkımız yok.** Bu yüzden:

* Dosyalar `.gitignore` ile depo dışında tutulur, yerelde kalır.
* Yayımlanan belgelerde modellere **link ve atıf** verilir, dosyanın
  kendisi konmaz.
* Yeniden dağıtmak istersen kaynak depo sahibinden izin al (issue açmak
  yeterli); izin gelirse `.gitignore` satırı kaldırılır ve verilen lisans
  buraya yazılır.

Bu projenin kendi ürettiği mekanik belgeler (`hardware/drawings/`, ölçü
tabloları, `config.h` içindeki geometri) bu kısıttan bağımsızdır ve
projenin lisansı altındadır.

## Dosyalar

| Dosya | Parça |
|---|---|
| `Crank.STL` | Krank — mil merkezi–pim arası 31 mm |
| `Coupler.STL` | Biyel — pim merkezleri arası 90 mm |
| `Beam Hinge.STL` | Beam mafsalı (p2 tarafı, sabit) |
| `Beam End.STL` | Beam'in biyele bağlanan ucu |
| `Hinge Tower.STL` | Mafsal kulesi |
| `Motor Tower.STL` | Motor kulesi |
| `Encoder Cover.STL` | AS5600 kapağı |
| `BALL AND BEAM ASSEMBLY.zip` | SolidWorks parça ve montaj dosyaları |

Bu düzenek kamerayla konum ölçtüğü için kaynak depodaki potansiyometre ve
araba (cart) parçaları indirilmedi.
