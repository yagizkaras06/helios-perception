# Geliştirme düzeneğinde tespit edilenler

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

5. **Ethernet bağlantısı / IP.** LiDAR ağında DHCP yoktur; bilgisayarın arayüzüne sabit bir adres
   (sürücü yapılandırmasındaki hedef adres) verilmelidir. NetworkManager'ın DHCP profili fiziksel bağlantı
   koptuğunda elle eklenmiş adresi siliyordu (belirti: sürücüde `ERRCODE_MSOPTIMEOUT`, arayüzde saniyede
   ~1 ARP paketi). Kalıcı çözüm: arayüz için statik bir NetworkManager profili.
6. **Olay kaydedicisi ayrı süreçte** ~1 MB'lık bulutlara reliable abone olunca sürücüyü tıkayıp 1–2 s'lik
   veri kesintilerine yol açtı; bu yüzden `record_events:=true` dedektör ve kaydediciyi tek süreçte
   (`perception_node`) başlatır.
7. **Paylaşılan Wi-Fi ağında ROS 2 keşfi.** Aynı ağda, aynı `ROS_DOMAIN_ID` ile çalışan başka bir
   bilgisayarın düğümleri (ör. `/rviz`, `/robot_state_publisher`) bu makinede görünüyordu. Varsayılan
   keşif ayarıyla (`SUBNET`) sürücünün `/rslidar_points` yayını 2–7 Hz'e düştü. Tüm yığın
   `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` ile yeniden başlatılınca 10 Hz'e döndü. Yeniden başlatmanın
   kendisinin de etkisi olabilir (8. madde), bu yüzden neden-sonuç kesin değil. Yine de paylaşılan
   ağlarda yerel keşif önerilir.
8. **Link kopması sonrası sürücü çıkışında kare kaybı.** Ethernet bağlantısı kopup geri geldikten sonra
   ağ arayüzü saniyede ~1500 paketi eksiksiz alıyordu (Helios 32 için tam hız). Buna karşın sürücünün
   yayınladığı taramalarda boşluklar vardı: başlık damgalarında atlamalar, dedektöre ~5 Hz. Dedektör
   bunu `SENSOR_DEGRADED` / `NO_DATA` olarak bildirdi. Sürücünün yeniden başlatılması bir önceki
   seferde 10 Hz'i geri getirdi. Kaynak sürücü tarafında; bu paket düzeltemez, yalnızca raporlar.
