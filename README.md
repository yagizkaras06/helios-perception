# helios_obstacle_detection (v0.2.0)

RoboSense Helios 32 için ROS 2 Jazzy / C++17 **yalnızca algılama + tahmin** yığını:
gelişmiş zemin segmentasyonu, menzile uyarlanabilir kümeleme, yönlü 3D kutular, **yalnızca LiDAR
ile ego-hareket kestirimi**, Kalman/Hungarian **çoklu nesne takibi**, hareketli/sabit ayrımı, hız ve
yön tahmini, belirsizliğe duyarlı **TTC / risk seviyesi**, sensör veri kaybı ve gecikme analizi,
`/diagnostics`, RViz2 görselleştirme, rosbag2 kayıt/oynatma, otomatik testler ve ölçüm araçları.

> **Araç kontrolüne bağlı değildir.** Gaz / fren / direksiyon / `cmd_vel` yayınlamaz, hiçbir
> denetleyiciyle konuşmaz (launch testi bunu otomatik doğrular). Risk seviyeleri yalnızca algılama
> çıktısıdır. Kamera, GPS, IMU, tekerlek odometrisi **kullanılmaz** — yalnızca `/rslidar_points`.
> `rslidar_sdk`, IP adresleri ve ağ yapılandırması değiştirilmedi.

## İçindekiler
1. [Mimari](#mimari) · 2. [Algoritmalar](#algoritmalar) · 3. [Belirsizlik ve sınırlar](#belirsizlik-ve-sınırlar)
4. [Topic'ler ve mesajlar](#topicler-ve-mesajlar) · 5. [Derleme / çalıştırma / test](#derleme--çalıştırma--test)
6. [Kayıt ve tekrar oynatma](#kayıt-ve-tekrar-oynatma) · 7. [Ölçülen sonuçlar](#ölçülen-sonuçlar)
8. [Bu kurulumda tespit edilenler](#bu-kurulumda-tespit-edilenler) · 9. [Ayar ipuçları](#ayar-ipuçları)

## Mimari

```
rslidar_sdk ──/rslidar_points (PointCloud2 32x1800, XYZI, ~10 Hz)
   │ abonelik: reliable, KeepLast(1) → tek-slot posta kutusu (yalnızca en yeni kare, birikme yok)
   ▼ İŞÇİ İŞ PARÇACIĞI ─ PerceptionPipeline (ROS'tan bağımsız, testler/benchmark aynı kodu çalıştırır)
   │  CloudProcessor     tek geçiş: alan doğrulama + NaN/Inf + menzil + ROI + öz-filtre + azimut kapsaması
   │  voxel grid         yüke uyarlanabilir yaprak boyutu (bütçe aşılırsa kabalaşır, rahatlayınca geri döner)
   │  CzmGroundSegmenter eşmerkezli bölge/halka/sektör hücreleri, hücre başına PCA (paralel), adaptif eşikler
   │        ├──► ObstacleClusterer (menzile uyarlanabilir) → AABB + L-şekli OBB (paralel)
   │        └──► EgoMotionEstimator (thread-pool görevi, eşzamanlı): düzlemsel nokta-düzlem ICP
   │  MultiObjectTracker CV-Kalman + gating + Hungarian, yaşam döngüsü, hareket sınıflandırması
   │  assessRisk         TTC (nominal + kötümser), CPA, çarpışma rotası, risk seviyesi
   ├──► /helios/obstacles, /helios/tracks   (birincil çıktılar, önce yayınlanır)
   └──► GÖRSELLEŞTİRME İŞ PARÇACIĞI (en yeni kare; yalnızca abone varsa): marker'lar + hata ayıklama bulutları
EXECUTOR: 1 Hz istatistik/izleme zamanlayıcısı + diagnostic_updater (/diagnostics)
```

| Modül | Dosya |
|---|---|
| İş parçacığı havuzu (kilitlenmesiz `parallelFor`) | `thread_pool.{hpp,cpp}` |
| Bulut dönüştürme / filtre / azimut kapsaması | `cloud_processor.*` |
| Zemin: CZM (varsayılan) / bölgesel RANSAC (v0.1) | `czm_ground_segmenter.*`, `ground_segmenter.*` |
| Kümeleme (sabit / menzile uyarlanabilir) | `obstacle_clusterer.*` |
| AABB + L-şekli yönlü kutu | `bounding_box.*` |
| LiDAR ego-hareket | `ego_motion_estimator.*` |
| Kalman, Hungarian, takipçi | `kalman_filter.hpp`, `hungarian.*`, `multi_object_tracker.*` |
| TTC / risk | `collision_risk.*` |
| Sensör sağlığı, gecikme, CPU/RSS | `sensor_health.*`, `resource_monitor.*`, `performance_monitor.*` |
| Pipeline (ROS'suz) | `perception_pipeline.*` |
| RViz marker'ları | `marker_builder.*` |
| Sentetik Helios-32 + CLEAR-MOT metrikleri | `lidar_simulator.*`, `tracking_metrics.*` |
| ROS düğümü / benchmark | `detection_node.cpp`, `benchmark_main.cpp` |

**Bellek:** zemin segmentasyonunda sayma sıralaması (hücre başına tahsis yok), yeniden kullanılan
tamponlar, `thread_local` çalışma alanları, sıfır kopya paylaşılan bulutlar, hata ayıklama bulutları
ve marker'lar yalnızca abone varken üretilir.

## Algoritmalar

**Zemin (CZM, Patchwork++ esinli).** Yatay düzlem menzil bölgelerine → halkalara → sektörlere
bölünür (uzakta daha iri hücreler). Her hücrede en alçak noktalardan tohum + yinelemeli PCA düzlemi
(paralel). Hücre kabul testleri, içten dışa: diklik (normal ≤ 15°), düzlemsellik (çizgi benzeri
destek reddi), düzlük (en küçük özdeğer, adaptif eşik), yakın halkalarda montaj yüksekliği önceliğine
göre yükseklik (adaptif eşik), aynı yöndeki kabul edilmiş iç hücreye göre **merkez ve 4 köşede**
süreklilik. Masa/platform/araç tavanı gibi yüksek düz yüzeyler zemin sayılmaz. Uzak hücrelerde iç
referans zorunludur: Helios 32 0.9 m yükseklikte düz zeminde 15.4–25.6 m arasına hiç zemin ışını
düşürmez, orada tek başına bulunan "düzlem" genellikle bir nesnenin tek ışın halkasıdır.
Reddedilen/seyrek hücreler en yakın kabul edilmiş hücrenin düzlemiyle sınıflanır; o da yoksa
yapılandırılmış öncül düzlem kullanılır ve **raporlanır** (`RELIABLE / DEGRADED / FAILED`).

**Kümeleme.** Tolerans menzille büyür: `tol(r) = clamp(max(0.45, r·tan 1.5°), 0.45, 1.0)` (32 ışınlı
sensörde ışın aralığı menzille doğrusal artar; sabit tolerans uzaktaki nesneyi şeritlere böler).
**Yönlü kutu:** yakınlık kriterli arama tabanlı dikdörtgen uydurma (Zhang ve ark. 2017) — kısmen
görünen (L-şekilli) araçlarda sağlam.

**Ego-hareket (yalnızca LiDAR).** Ardışık zemin-dışı bulutlar düzlemsel (x, y, yaw) nokta-düzlem ICP
ile hizalanır. Dinamik nesnelere karşı üç önlem: (1) önceki karede hareketli izlerin bölgeleri
dışlanır, (2) Cauchy çekirdeği, (3) **hipotez uzlaşısı** — ICP optimumu, sıfır-hareket ve sabit-hız
hipotezleriyle "geometrik olarak tutarlı nokta sayısı" üzerinden karşılaştırılır; daha basit hipotez
neredeyse aynı desteği alıyorsa o seçilir (hareketli kalabalık sabit sensörü "sürükleyemez").
Gauss-Newton Hessian'ından kovaryans (`σ²H⁻¹`) ve dejenerelik (koridor, açık alan) hesaplanır.
Durumlar: `STATIONARY / MOVING / UNRELIABLE (kısa süreli sabit-hız tahmini, artan kovaryans) / UNKNOWN`.

**Takip.** Yerel odometri çerçevesinde CV Kalman (Joseph formu). Ölçüm gürültüsü menzil, seyrek küme,
nesne boyutu ve **o karenin ego-artışının belirsizliği** (yaw hatası × kaldıraç kolu) ile büyür.
Ki-kare + metrik geçitli, `NIS + ln|S|` maliyetli global Hungarian atama. TENTATIVE → CONFIRMED
(3 isabet) → COASTING → silme. Kutu boyutu ani değişirse (kısmi örtme / birleşme) ölçüm gürültüsü
büyütülür ve merkez, hareket testine katılmaz.

**Hareketli / sabit ayrımı** iki bağımsız kanıt + histerezis ister:
* pencere içi (1 s) **merkez regresyonu**: hız > 0.4 m/s ve > 3σ ve yer değiştirme > 0.3 m;
* **nokta-yüzey tutarlılığı**: izin şimdiki noktaları ile ~0.5 s önceki noktaları arasındaki 2B en
  yakın komşu mesafelerinin medyanı (odometri çerçevesinde). Statik bir nesnenin yüzeyleri hangi yüzü
  görünürse görünsün çakışır → hareket eden sensörde kutu merkezinin bakış açısı kaynaklı kaymasına
  bağışıktır. (Bu, sentetik hareketli-sensör senaryosunda yanlış "MOVING" oranını %7.9'dan %0'a indirdi.)
* Kanıt yoksa sonuç **UNKNOWN** kalır; ego-hareket bilinmiyorsa tüm izler anında UNKNOWN olur.

**Statik arka plan modeli ("10 s kıpırdamayan statiktir").** Zemin dışı noktalar odometri
çerçevesinde 0.15 m'lik voxel'lerde biriktirilir; en az `static_map.static_time_s` (10 s) boyunca
karelerin ≥%50'sinde dolu kalan voxel'ler **arka plan** olur, 5 s görülmeyenler unutulur. Sensör
sabitken her tarama arka plan / ön plan noktalarına ayrılıp **ayrı ayrı kümelenir** (duvarın
yanındaki kişi duvardan ayrılır); arka plan kümeleri STATIC, geri kalan her şey dinamiktir:
hareket edenler MOVING, yeni gelen / duran ama 10 s'yi doldurmamış olanlar UNKNOWN. 10 s
kıpırdamayan kişi statiğe döner, yürümeye başlayınca hemen yeniden dinamik olur. Açılıştan sonraki
ilk 10 s'de her şey dinamik görünür ("learning static background"). Sensör hareket ederken
nesneler kısa süre ve değişen açılardan görüldüğü için bu kural uygulanmaz, kanıt tabanlı
sınıflandırma kullanılır. Işın izleme yok: statik bir nesnenin boşalttığı yer en fazla 5 s arka plan
kalır.

**TTC / risk.** LiDAR merkezli `ego_radius` dairesine göre: kapanma hızı `c = -u·v_rel`,
`TTC = açıklık / c`, **kötümser TTC** `= açıklık / (c + 2σ_c)`, CPA (en yakın yaklaşma) ve 2σ yanal
bant ile çarpışma rotası. `v_rel = v_nesne − v_ego − ω×p`, kovaryansı nesne + ego (+ kaldıraç) toplamı.
STATIC izlerde nesne hızı 0 (±0.05 m/s) modellenir. Hızı güvenilir olmayan izlerde (genç iz, büyük
yapı, ego bilinmiyor) TTC hesaplanmaz, risk yalnızca mesafeye dayanır (`ttc_valid=false`).
Seviyeler: `NONE / LOW / MEDIUM / HIGH / CRITICAL`.

**Sensör sağlığı / veri kaybı.** Başlık damgalarından kare boşluğu ve kaçırılan kare tahmini, zaman
sıçraması (bag döngüsü, sürücü yeniden başlatma → takipçi/ego sıfırlanır), geçerli dönüş oranı
(kirli/kapalı pencere), 10°'lik azimut kapsaması (paket kaybı/örtme), ani nokta düşüşü, veri zaman
aşımı (`NO_DATA`, marker'lar temizlenir, eski veri yeniden yayınlanmaz).

**Gecikme.** Ana bilgisayar tarafı: alım → işçi başlangıcı (`queue_wait`), aşama süreleri, alım →
birincil çıktı yayını (`rx_to_publish`). Sensör saati senkron değilse (PTP/GPS yok) mutlak
sensör→ana bilgisayar gecikmesi **ölçülemez**; düğüm bunu tespit eder ve yalnızca teslim jitter'ını
raporlar (bkz. [tespit edilenler](#bu-kurulumda-tespit-edilenler)).

**Uyarlanabilirlik.** CZM yükseklik/düzlük eşikleri kayan ortalama+σ ile; voxel boyutu işlem
süresi bütçesine göre (`adaptive.*`); kümeleme toleransı menzile göre.

## Belirsizlik ve sınırlar

LiDAR'ın tek başına güvenilir sağlayamadığı sonuçlar kesinmiş gibi sunulmaz:

* Hızlar **zemine göre**dir ancak yalnızca `velocity_reliable=true` iken güvenilir. Her hız
  `speed_sigma`, `velocity_covariance` ve yön `heading_sigma` ile gelir; yön yalnızca MOVING izlerde
  geçerlidir.
* Ego-hareket yalnızca LiDAR'dan gelir: düzlemsel model (roll/pitch/z yok), nokta zaman damgası
  olmadığı için tarama içi bozulma düzeltmesi yok (10 m/s'de tarama ~1 m yayılır), çok seyrek veya
  dejenere sahnelerde (tek duvar, uzun koridor, açık alan) `UNRELIABLE/UNKNOWN` olur ve bu durumda
  hareket sınıfları UNKNOWN'a döner. ~0.3 m/s altındaki ego "sürünme"si STATIONARY görülebilir
  (uzlaşı eşiği 6 cm/kare).
* TTC sensör merkezli bir daireye göredir (araç gövdesi bu pakette bilinmiyor, `risk.ego_radius`).
  Sabit göreli hız varsayımı; manevra eden nesnelerde kötümser TTC kullanın.
* Kutu merkezi, nesnenin görünen kısmının merkezidir; örtme ve bakış açısı değişimi bunu kaydırır
  (gürültü modelinde ve sınıflandırmada hesaba katılır, ancak yok edilemez).
* Sınıf (yaya/araç) tahmini yapılmaz.
* Sentetik doğruluk sayıları basitleştirilmiş bir simülatörden gelir (düz zemin, tarama içi hareket
  yok, yaklaşık ışın düzeni): gerçek dünya için **üst sınır** olarak okuyun. Gerçek etiketli veri
  olmadan gerçek algılama doğruluğu ölçülemez; bag üzerinde yalnızca tutarlılık metrikleri raporlanır.

## Topic'ler ve mesajlar

Varsayılan namespace `helios`. Tüm çıktılar giriş başlığını (ns damga + `frame_id`) taşır.

| Topic | Tip | Not |
|---|---|---|
| `/rslidar_points` (giriş) | sensor_msgs/PointCloud2 | sürücüden; yeniden yayınlanmaz |
| `/helios/tracks` | helios_obstacle_detection/TrackedObjectArray | **asıl çıktı**: kalıcı ID, kutu, hız±σ, yön, TTC, CPA, risk, ego durumu |
| `/helios/obstacles` | helios_obstacle_detection/ObstacleArray | kare bazlı tespitler (AABB + OBB) |
| `/helios/track_markers` | visualization_msgs/MarkerArray | risk renkli OBB, `ID / MOV-STAT / hız±σ / yön / mesafe / TTC[kötümser] / risk` etiketi, hız oku, tahmini göreli yörünge, 2σ elips, iz, ego ayak izi + hız, durum paneli |
| `/helios/obstacle_markers` | MarkerArray | kare bazlı AABB (v0.1 uyumlu) |
| `/helios/filtered_points`, `ground_points`, `obstacle_points`, `obstacle_clusters` | PointCloud2 | hata ayıklama (kümeler risk renkli) |
| `/helios/processing_stats` | helios_obstacle_detection/ProcessingStats | 1 Hz: FPS, CPU, RSS, gecikme, veri kaybı, saat senkronu, zemin, ego, takip, aşama süreleri |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | `LiDAR input`, `Scan quality`, `Time synchronization`, `Processing`, `Ground segmentation`, `Ego motion`, `Tracking` (`rqt_robot_monitor` ile izlenebilir) |

RViz'de varsayılan olarak **STATIC (arka plan) izler çizilmez**; MOVING ve UNKNOWN (dinamik ama
şu an hareketsiz) izler çizilir (`visualization.show_static`, `visualization.show_unknown`).
STATIC izler `/helios/tracks` çıktısında ve risk hesabında yer almaya devam eder. RViz renk anahtarı: açık mavi NONE, yeşil LOW, sarı MEDIUM, turuncu HIGH, kırmızı CRITICAL;
yarı saydam = COASTING (tahmin). Etiketler ASCII (RViz yazı tipinde Türkçe karakter yok).

## Derleme / çalıştırma / test

```bash
cd ~/helios_ws/src
source /opt/ros/jazzy/setup.bash
colcon build --packages-select helios_obstacle_detection
source install/setup.bash

ros2 launch helios_obstacle_detection detection.launch.py              # yalnız algılama
ros2 launch helios_obstacle_detection detection.launch.py rviz:=true   # + RViz2
ros2 run helios_obstacle_detection obstacle_report.py                  # canlı tablo (izler + istatistik)
ros2 run helios_obstacle_detection perf_report.py --duration 60 --out ~/perf.md   # FPS/CPU/gecikme raporu
ros2 topic echo /diagnostics

# testler (6 gtest paketi + ROS launch testi); ROS_DOMAIN_ID ile canlı sistemden yalıtın
ROS_DOMAIN_ID=77 colcon test --packages-select helios_obstacle_detection && colcon test-result --verbose

# çevrimdışı ölçüm
ros2 run helios_obstacle_detection helios_benchmark synthetic                   # yer gerçekli doğruluk
ros2 run helios_obstacle_detection helios_benchmark synthetic --ego oracle      # mükemmel ego ile üst sınır
ros2 run helios_obstacle_detection helios_benchmark bag ~/helios_ws/bags/helios_test_20s --csv frames.csv
```

Testler: Hungarian (kaba kuvvete karşı), Kalman yakınsaması/PSD, takipçi (kesişen nesnelerde ID,
hareketli sensörden statik nesne, coasting/silme/zaman sıçraması), ego-hareket (hareket kurtarma,
sabit sensör, koridor dejenereliği, boşluk/zaman aşımı), CZM (kutu, eğim, yüksek masa, başarısızlık,
adaptif eşik, paralel=sıralı), TTC/risk, sağlık (boşluk, sıçrama, saat senkronu, kapsama), iş
parçacığı havuzu (istisna, kilitlenmezlik), OBB, uyarlanabilir kümeleme, simülatörle uçtan uca
sistem testleri (MOTA, ID değişimi, hız, sınıflandırma, TTC, kare düşmesi) ve **düğümün hiçbir
aktüasyon konusu açmadığını** doğrulayan launch testi.

VS Code (snap) terminalinden RViz2 açılırken `libpthread ... __libc_pthread_init` hatası alınırsa
RViz2'yi normal bir terminalden başlatın.

## Kayıt ve tekrar oynatma

Bu makinedeki `ros2 bag record/play` (rosbag2 CLI) `Couldn't parse log level: '--log-level info'`
hatasıyla başlamıyor (veri değil CLI hatası; `sudo apt upgrade` ile rosbag2 güncellemesi muhtemelen
düzeltir — sistem paketlerine dokunulmadı). Bu yüzden `rosbag2_py` tabanlı `bag_tool.py` kullanılır
(MCAP, `ros2 bag info` ile okunabilir):

```bash
# ham LiDAR
ros2 run helios_obstacle_detection bag_tool.py record -o ~/helios_ws/bags/kayit1 --duration 30
# ham LiDAR + tüm algılama çıktıları + diagnostics
ros2 run helios_obstacle_detection bag_tool.py record -o ~/helios_ws/bags/kayit2 --preset perception
ros2 run helios_obstacle_detection bag_tool.py info ~/helios_ws/bags/kayit1
# algılama çalışırken kayıt
ros2 launch helios_obstacle_detection detection.launch.py record:=true record_dir:=~/helios_ws/bags/kayit3

# canlı sistemle KARIŞMADAN tekrar oynatma: /bag/rslidar_points + ayrı namespace (helios_bag)
ros2 launch helios_obstacle_detection replay.launch.py bag:=$HOME/helios_ws/bags/helios_test_20s rviz:=true loop:=true
```

## Ölçülen sonuçlar

Makine: 16 çekirdek, 31 GB RAM, Ubuntu 24.04, ROS 2 Jazzy, Release (`-O3`), `threads: 2`.
Ayrıntılı tablolar: [`doc/`](doc/).

**Gerçek zamanlı (kayıtlı Helios bag'i 10 Hz'de döngüde oynatılarak, 45 s, varsayılan ayarlar)** —
[`doc/live_replay_perf.md`](doc/live_replay_perf.md)

| Ölçüt | Değer |
|---|---|
| Giriş / çıkış FPS | 10.03 / 10.03 Hz, 0 atlanan, 0 aşırı yük |
| CPU (tüm düğüm, %100 = 1 çekirdek) | **%21.3** ort. (düğüm raporu) — harici `/proc` ölçümü: %21.2 |
| Bellek | RSS 67 MB, 19 iş parçacığı |
| Gecikme: alım → `/tracks` yayını | **13.8 ms ort., 15.5 ms p95, 23.4 ms maks** |
| Aşamalar (ort. ms) | ROI 1.0 · voxel 2.2 · zemin 0.5 · kümeleme 9.0 · kutu 0.5 · ego 5.8 (eşzamanlı) · takip+risk 0.2 |
| Sensör→ana bilgisayar gecikmesi | **ölçülemez**: LiDAR saati senkron değil (bkz. aşağı); teslim jitter'ı p95 13.9 ms |

**Çevrimdışı (aynı bag, olabildiğince hızlı, 3 tur)** — [`doc/benchmark_bag_czm.md`](doc/benchmark_bag_czm.md):
pipeline 4.9 ms ort. / 5.1 ms p95, ~205 kare/s, kare başına 7.2 ms CPU. Gerçek zamanlı süreler
~2.8× daha uzun; düşük yükte CPU frekans ölçeklemesi ve soğuk önbellek etkisi (10 Hz'de çekirdekler
boşta bekliyor). 1 vs 15 işçi: 1–2 işçi bu nokta yoğunluğunda en hızlı ve en düşük jitter'lı
(büyük havuzun uyandırma maliyeti kazancı aşıyor) — varsayılan `threads: 2`.

**Gerçek bag tutarlılığı** (etiket yok → doğruluk değil, tutarlılık): ego 600/603 kare STATIONARY
(sensör gerçekten sabit; 3 kare = her turun ilk karesi), statik izlerin hız gürültüsü 0.00 m/s,
kare başına ~30 onaylı iz, iz ömrü medyanı 20 s (= kaydın tamamı; RANSAC zemini ile 11.7 s), sahte
"MOVING" yok.

**Sentetik doğruluk (ışın izlemeli Helios-32 benzeri tarama, yer gerçeği, 10 s / senaryo)** —
[`doc/benchmark_synthetic_lidar_ego.md`](doc/benchmark_synthetic_lidar_ego.md)

| Senaryo | MOTA | MOTP | Kesinlik / Duyarlılık | ID değ. | Hız RMSE / hız MAE | Yön MAE | Hareketli→MOVING | Statik→MOVING | TTC MAE |
|---|---|---|---|---|---|---|---|---|---|
| Sabit sensör, yayalar + araçlar | 0.945 | 0.19 m | 0.975 / 0.975 | 4 | 0.26 / 0.08 m/s | 2.2° | 322/335 (UNKNOWN 13) | **0 / 362** | 0.41 s |
| Sensör 3 m/s + dönüş, 40 statik engel | 0.839 | 0.29 m | 0.948 / 0.899 | 28 | 0.50 / 0.11 m/s | 2.6° | 186/253 (STATIC 10) | **0 / 1813** | 0.28 s |
| Sabit sensör, nesne 2 m/s ile üzerine | 0.992 | 0.17 m | 0.992 / 1.000 | 0 | 0.01 / 0.00 m/s | 0.2° | 91/93 | 0 / 280 | **0.02 s** |

* Ego-hız hatası (yalnızca LiDAR): sabit sensörde 0.000 m/s, 3 m/s + 0.05 rad/s dönüşte 0.022 m/s.
  "Mükemmel ego" (`--ego oracle`) ile sonuçlar aynı → kalan hatalar ego kaynaklı değil, kümeleme /
  örtme kaynaklı ([`doc/benchmark_synthetic_oracle_ego.md`](doc/benchmark_synthetic_oracle_ego.md)).
* Kötümser TTC gerçek TTC'den büyük kalma oranı: %9.1 / %5.8 / %0.
* Bilinen kalan hatalar: yakındaki statik nesneyle aynı kümeye düşen yaya (10 kare STATIC),
  örtmeden sonra yeniden başlayan izler (ID değişimi), uzak/arkadaki küçük engellerin kaçırılması.
* CZM vs RANSAC (düz sentetik dünyada RANSAC'ın ideal durumu): eşdeğer (0.839 vs 0.841 MOTA);
  gerçek bag'de CZM iz ömrü belirgin daha uzun. Eğimli/engebeli zeminde CZM'nin üstünlüğü birim
  testlerle gösterildi, sahada etiketli veri ile doğrulanmalı.

## Bu kurulumda tespit edilenler

1. **LiDAR saati senkron değil.** `use_lidar_clock: true`; başlık damgaları ana bilgisayar saatinden
   ~311.9 milyon s (≈9.9 yıl) geride (PTP/GPS yok). Takip ve TTC için sorun değil (kare aralıkları
   0.1000 s ile çok düzenli), ancak mutlak sensör→ana bilgisayar gecikmesi ölçülemez ve damgalar
   diğer sensörlerle eşleştirilemez. Düğüm bunu `/diagnostics` → `Time synchronization` (WARN) ile
   bildirir. (Sürücü yapılandırması değiştirilmedi.)
2. **Sensörün etrafında ~1.7×1.4 m'lik bir yapı** (0.5–1 m mesafede, zeminden ~0.6–1.1 m yükseklikte,
   muhtemelen üzerinde durduğu masa/platform) kayıttaki karelerin %99'unda `CRITICAL` riske neden
   oluyor. Bu gerçek bir fiziksel nesne; araçta `filter.self_filter` kutusunu gövdeye göre ayarlayın.
3. **Zemin `DEGRADED`:** kapalı alanda zemin yalnızca sensörün arkasında küçük bir yayda görünüyor;
   noktaların >%50'si öncül düzlemle sınıflanıyor. Bu dürüst bir durum bildirimidir, hata değildir.
4. `ros2 bag` CLI hatası (yukarıda); `bag_tool.py` ile çözüldü.
5. Eski (v0.1) dedektör `/helios` altında hâlâ çalışıyordu; yedek: `~/helios_ws/backup/`.

## Ayar ipuçları (`config/detection.yaml`)

* `ground.expected_height`: montaj yüksekliğinin eksi değeri (şu an −0.90 m); eğik montajda
  `expected_normal`.
* Sabit test tezgâhında ego-hareket kestirimi yerine `ego_motion.mode: assume_stationary`
  kullanılabilir (araç üzerinde **kullanmayın**).
* Yoğun dış ortam / daha fazla nokta: `threads: 0` (otomatik) ve `adaptive.target_ms` ile bütçe.
* Yanlış "hareketli" kararları için: `tracking.moving_speed`, `motion_sigma_factor`,
  `point_evidence_age_s`; daha temkinli risk için `risk.sigma_factor`, `risk.ego_radius`.
* Zemin yöntemi: `ground.method: czm | ransac_zones`.
