# Helios Perception v0.3.0-dev — rapor

Geliştirme dalı: `v0.3.0-dev` (`~/helios_ws/dev/v030_ws/src/helios_obstacle_detection`).
Baseline: `v0.2.1` etiketi. Canlı kurulum (`~/helios_ws/src/install`) değiştirilmedi.
Ölçüm betikleri `scripts/` altında (`run_eval.sh`, `pseudo_gt*.py`, `aggregate.py`); ham sonuçlar `doc/v030_eval/`.

## Ne değişti

| Bileşen | v0.3 |
|---|---|
| Yaşam döngüsü | TENTATIVE → CONFIRMED ⇄ OCCLUDED → LOST → silinir. OCCLUDED = tahmin (`predicted`, büyüyen kovaryans). Uzun saklama (2.5 s) yalnızca açıklanmış tıkanmada ve dinamik izlerde; görüş hattı boşsa 0.8 s. LOST izler yayınlanmaz, 2 s yeniden tanıma için tutulur. |
| Tıkanma testi | Tahmini kutunun azimut aralığında, nesnenin ön yüzünden yakın engel (1° bölmeler), birleşmiş küme, menzil dışı → `occlusion_reason`. |
| Kısmi görünüm | Küçülmüş kutu, merkezi bir aralığa kısıtlar (merkezi ve hızı görünen parçaya çekmez). Yalnız hareketli izler, aralık ≤ 1 m. |
| Birleşme / ayrılma | Eşleşemeyen yerleşik izin tahmini kutusu başka bir kümenin içindeyse küme noktaları tahmini kutulara göre bölünür (en çok 3 iz, her izin kutusunda nokta şart). |
| İlişkilendirme | Hungarian korunur; maliyet = NIS + ln\|S\| + boyut (log oranı) + (1 − BEV IoU) + hareketli iz ↔ arka plan kümesi cezası; ki-kare + metrik geçitleme. |
| Statik harita | İz güdümlü: son 10 s içinde hareket etmiş izlerin kümeleri statik süre biriktirmez ("10 s hareketsiz → statik" hareketin bittiği andan sayılır). |
| Hareket | Ölçüm yokken hareket durumu dondurulur. Pencere içinde değişim noktası (menteşe) testi (5σ, ≥0.2 m, 3 örnek, tutarlı yön). Eşikler (0.3/0.2 m/s, 2 kare onay) korundu. 2 s kanıtsız MOVING → UNKNOWN. |
| Belirsizlik | `velocity_status` RELIABLE/UNRELIABLE/UNKNOWN; ego UNKNOWN → `relative_motion_only`; ego UNRELIABLE → yeni mutlak karar yok; tahmin/birleşik/σ>0.6 m/s → TTC yalnızca mesafe tabanlı. `track_confidence`, `motion_confidence` (sezgisel, olasılık değil). |
| ROS | `TrackedObject`: yeni alanlar (geri uyumlu; `TRACK_COASTING` = `TRACK_OCCLUDED`). `ProcessingStats`: occluded/lost/re-id/merge sayaçları, tahmin/ilişkilendirme/tıkanma süreleri. |
| RViz | Kutu rengi: MOVING kırmızı, UNKNOWN sarı; OCCLUDED kesikli + yarı saydam; güvenilmez hız `v? x±σ`; etikette yaşam döngüsü + güven. STATIC gizli (risk hesabında). |

## Gerçek kayıt (bags/walk_001258, sabit sensör, 90 s)

Referans etiket yoktur. İki ölçüt:
1. **v0.2 proxy** (önceki rakamlarla karşılaştırma için; referansı izleyicinin kendi konumlarından alır → döngüsel).
2. **Sözde-referans (izleyiciden bağımsız)**: tüm kayıt boyunca 3B voksel arka planı + insan boyutlu ön plan kümeleri + çevrimdışı bağlama/yumuşatma. Yalnızca başka dinamik nesneden ≥1.5 m uzak "temiz" kareler; 10 s hareketsiz (arka plana katılmış) nesneler ve sensörün <1 m yakını (masa bölgesi) ayrı raporlanır. İnsan doğrulaması yapılmadı (önerilen yöntem: bu aday yörüngelerin BEV görüntüleri üzerinden elle onaylanması / düzeltilmesi).

| Metrik | v0.2.1 | v0.3.0-dev |
|---|---|---|
| Yürüme anlarında MOVING (proxy, v>0.4) | %61 | %64 |
| Dururken yanlış MOVING (proxy) | %0.2 (5/2067) | %0.04 (1/2311) |
| Medyan yön hatası (proxy) | 9.0° | 8.8° |
| Yürüme anlarında MOVING (sözde-referans) | %74.1 | %77.2 |
| Statik sahnede yanlış MOVING (sözde-referans) | %0.21 | %0.12 |
| Medyan yön hatası (sözde-referans) | 10.7° | 10.4° |
| Recall (≥1 m, temiz) | %100 | %100 |
| ID switch / IDF1 (temiz bölümler) | 2 / 0.992 | 4 / 0.976 |
| Masa bölgesi (<1 m) recall | %44 | %50 |

ID switch notu: v0.3'te kişi kaydın tamamında tek kimlikle (42) izlendi; v0.2'de aynı kişi 3 kimlik aldı (43 → 60 → 265). v0.3'teki ek switch'ler sözde-referansın kişi/sandalyeyi ayıramadığı 80–85 s bölümünde ve bir parça izde. Kayıtta yalnızca 3 "dur → yürü" olayı var; gerçek veride gecikme için istatistiksel sonuç çıkarılamaz.

## Sentetik (kesin referans, 5 tohum/senaryo)

| Senaryo | ID switch (v0.2 → v0.3) | IDF1 | Yanlış MOVING (hiç hareket etmemiş) | Not |
|---|---|---|---|---|
| occlusion | 54 → 1 | 0.728 → 0.982 | %0.24 → %0 | parçalanma 42 → 0 |
| merge_split | 29 → 12 | 0.874 → 0.939 | %0 → %0 | |
| wall_walk | 0 → 0 | 0.995 → 0.997 | %0 → %0 | başlangıç 12/15 → 15/15 |
| start_stop | 27 → 19 | 0.763 → 0.872 | %0.40 → %0 | FP 1601 → 912 |
| crossing | 31 → 7 | 0.730 → 0.930 | %0.27 → %0 | |
| ego_motion (hareketli sensör) | 191 → 111 | 0.654 → 0.753 | **%0.06 → %0.18** | gerileme, aşağıda |
| approach | 0 → 0 | 0.995 → 0.990 | %0 → %0 | FP 35 → 75 |

Hareket başlangıç gecikmesi (`start_stop`, 10 tohum, 240 olay): medyan 400 → 300 ms (−%25), ortalama 517 → 407 ms (−%21).
Duyarlılık analizi: onay 1 kare → gerçek kayıtta yanlış MOVING %0.12 → %0.27; yer değiştirme 0.15 m → ego_motion yanlış MOVING %0.42. Bu yüzden mevcut eşikler korundu.

## Performans

| Metrik | v0.2.1 | v0.3.0-dev |
|---|---|---|
| Çevrimdışı işlem süresi ort / P50 / P95 / P99 / maks | 10.40 / 9.63 / 15.30 / 19.21 / 27.99 ms | 10.76 / 10.19 / 13.83 / 18.69 / 26.76 ms |
| Takip + risk aşaması (ort / P95) | 0.09 / 0.12 ms | 0.29 / 0.46 ms |
| ROS düğümü, 1× oynatma (ROS_DOMAIN_ID=77) | — | giriş 10.0 Hz, çıkış 10.0 Hz, 432/432 kare, 0 kayıp, CPU %26 (1 çekirdek), RSS 67 MB, alım→yayın ort 23.8 / P95 29.2 / maks 35.5 ms |

Not: düğüm ölçümü canlı sürücü + dedektör + RViz + Python oynatıcı aynı makinede çalışırken alındı. Takip içi ayrıntı süreleri (tahmin / ilişkilendirme / tıkanma) `/helios/processing_stats` içinde yayınlanır; çevrimdışı ayrı tablo üretilmedi.

## Testler

62 test, 0 hata: önceki 8 paket (50 test, launch testi dahil; aktüasyon topic'i yok) + `test_tracking_v03` (11 test: tek/beş kare kayıp, uzun kayıp + yeniden tanıma, birleşme/ayrılma, duvar önünde yürüme, kısa yürüyüş gecikmesi, duran nesne, ego belirsizliği, kimlik koruma, veri kesintisi, zaman sıçraması).

## Bilinen sınırlamalar

- Hareketli sensör senaryosunda (ego_motion) hiç hareket etmemiş nesnelerde yanlış MOVING %0.06 → %0.18 (5 koşuda ~9 kare): yayanın dağınık statik nesneyle birleşip ayrıldığı nadir durumlar.
- FP: tıkanmış dinamik izlerin tahminleri daha uzun gösterildiği için bazı senaryolarda arttı (occlusion 100 → 208, approach 35 → 75); start_stop ve merge_split'te azaldı.
- Masa bölgesi (<1 m) ve 10 s'den uzun hareketsiz kalan nesneler kural gereği arka plandır.
- Gerçek veride insan onaylı referans yok; gerçek "dur → yürü" olayı yalnızca 3.
- Güven değerleri sezgiseldir, kalibre edilmiş olasılık değildir.
