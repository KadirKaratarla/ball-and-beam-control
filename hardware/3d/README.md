# 3D basılı parçalar

Bu düzeneğin mekanik parçaları hazır bir açık projeden basıldı. Modeller bu
depoda **yeniden yayımlanmıyor**, kaynağından indirilmeli:

> **[IVProjects / Engineering_Projects — Ball and Beam Control System](https://github.com/IVProjects/Engineering_Projects/tree/main/ProjectFiles/Ball%20and%20Beam%20Control%20System)**

Kaynak deponun README'sinde yazan koşul:

> *"All files posted here are provided 'as is'. … You are free to use and
> modify all content posted here."*

Ayrı bir LICENSE dosyası yok. Dosyaları burada kopyalamak yerine kaynağa
bağlamak hem atfı doğru yerde tutuyor hem de parçalar güncellenirse en yeni
hâlini almanı sağlıyor.

## Bu projede kullanılan parçalar

| Dosya | Ne işe yarıyor |
|---|---|
| `Crank.STL` | Motor miline oturan krank |
| `Coupler.STL` | Biyel — krank pimi ile beam ucunu bağlar |
| `Beam Hinge.STL` | Beam mafsalı (p2, sabit uç) |
| `Beam End.STL` | Beam'in biyele bağlanan ucu (p1, hareketli) |
| `Hinge Tower.STL` | Mafsal kulesi |
| `Motor Tower.STL` | Motor kulesi |
| `Encoder Cover.STL` | AS5600 kapağı |
| `BALL AND BEAM ASSEMBLY.zip` | SolidWorks kaynak parçaları (değiştirmek istersen) |

Kaynak depodaki **potansiyometre ve araba (cart) parçalarına gerek yok**: bu
projede top konumu kameradan ölçülüyor, potansiyometreyle açı okunmuyor.

## Yazılımın beklediği ölçüler

Firmware'in kinematiği bu ölçülere göre çalışıyor
(`bb_esp32s3/src/config.h`):

| Sembol | Değer | Nereden |
|---|---|---|
| `r` | 31 mm | `Crank.STL` 43 mm uzunluğunda, uçları 6 mm yarıçaplı → pim merkezleri arası 31 mm |
| `l` | 90 mm | `Coupler.STL` 114 mm, uçları 12 mm yarıçaplı → 90 mm |
| `d` | 477 mm | Düzenekte ölçüldü (beam uzunluğu senin montajına bağlı) |
| pivot Δy | 90 mm | Mafsal pimi motor milinin bu kadar üstünde |

İlk ikisi doğrudan modellerin sınırlayıcı kutularından hesaplandı, yani
parçaları olduğu gibi basarsan tutar. **Parçaları değiştirirsen**
`LINK_CRANK_R_MM`, `LINK_ROD_L_MM`, `LINK_BEAM_D_MM` ve `LINK_PIVOT_DY_MM`
değerlerini güncelle; `BEAM_THETA_MAX_DEG` sınırı da bu geometriden
türetiliyor (bkz. [`../drawings/theta_vs_phi.svg`](../drawings/theta_vs_phi.svg)).

Ölçülü çizim: [`../drawings/linkage.svg`](../drawings/linkage.svg)
