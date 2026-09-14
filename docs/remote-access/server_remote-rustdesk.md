# RustDesk Távoli Asztal Útmutató (Telepítés & Használat)

Ez az útmutató bemutatja, hogyan tudsz távoli grafikus asztali kapcsolatot (GUI) létesíteni a gépek között a **RustDesk** nyílt forráskódú távoli asztal kliens segítségével Linux, Windows és macOS rendszereken.

> [!WARNING]
> **BeamNG.drive és járműszimuláció futtatására a RustDesk NEM alkalmas!**
> A 3D-s szimulátorokhoz és játékokhoz (mint a BeamNG) **Sunshine + Moonlight** kapcsolatot kell használni!

---

## 1. Mi az a RustDesk?

A RustDesk a TeamViewer és az AnyDesk biztonságos, nyílt forráskódú alternatívája. Lehetővé teszi:
* A teljes grafikus asztal elérését és vezérlését (egér, billentyűzet).
* Fájlátvitelt (drag-and-drop vagy beépített fájlkezelő).
* Vágólap szinkronizálást (szövegmásolás a két gép között).
* Közvetlen P2P vagy relé szerveren keresztüli titkosított kapcsolatot.

---

### ❌ Mire NEM jó a RustDesk?
* **3D szimulációk és játékok (pl. BeamNG.drive):** 
  * A RustDesk nem kezeli a raw mouse capture / relative mouse input módokat. Játék közben az egérkurzor eltűnhet, megakadhat a képernyő szélén, vagy a kamera irányíthatatlanul körbeforog.
  * Nem játékra optimalizált hardveres kódolást (NVENC) használ, így a képkockasebesség (FPS) alacsony, a beviteli késleltetés (input lag) pedig túl magas a precíz járműirányításhoz.
  * Nem kezeli natívan a fizikai kontrollereket és játékvezérlőket (DirectInput / XInput átvitel hiánya).
* **Hangátvitel komplex környezetben:** A többcsatornás, alacsony késleltetésű virtuális hangkimenetek kezelése bizonytalan.

### ✔️ Mire jó a RustDesk?
* **Rendszeradminisztráció és karbantartás:** Szerverbeállítások, grafikus konfigurációk és asztali kezelőfelületek elérése.
* **Általános irodai és fejlesztői munka:** Szövegszerkesztés, terminálok párhuzamos kezelése, IDE-k (pl. VS Code) használata.
* **Fájlátvitel és vágólap:** Kétirányú gyors fájlmásolás, szöveg és vágólap szinkronizálása a kliens és a távoli gép között.
* **Távsegítség (Remote Support):** Gyors grafikus belépés a bejelentkező felületre vagy az asztalra anélkül, hogy bonyolult hálózati beállításokat kellene konfigurálni a távoli félnél.

---

## 2. Telepítés operációs rendszerek szerint

### A) Linux (Ubuntu / Debian / Linux Mint)

1. **Csomag letöltése és telepítése:**
   Nyiss egy terminált, és töltsd le a legfrissebb `.deb` telepítőt:
   ```bash
   # Csomag letöltése
   wget -O rustdesk.deb https://github.com/rustdesk/rustdesk/releases/download/1.4.9/rustdesk-1.4.9-x86_64.deb

   # Telepítés és hiányzó függőségek automatikus feloldása
   sudo apt update
   sudo apt install -y ./rustdesk.deb
   ```

2. **Futtatás:**
   Indítsd el a Start menüből (*RustDesk*) vagy terminálból a `rustdesk &` paranccsal.

3. **Wayland vs. X11 megjegyzés:**
   A távoli irányítás Linuxon **X11** környezetben a legstabilabb. Ha Wayland-et használsz és fekete képernyőt kapsz, a bejelentkezési képernyőn válts át az X11 / Cinnamon / GNOME on Xorg munkamenetre.

---

### B) Windows (Windows 10 / 11)

1. **Letöltés:**
   * **Winget használatával (PowerShell):**
     ```powershell
     winget install --id RustDesk.RustDesk
     ```
   * **Vagy közvetlen letöltéssel:**
     Töltsd le a hivatalos telepítőt:
     [https://github.com/rustdesk/rustdesk/releases](https://github.com/rustdesk/rustdesk/releases)

2. **Telepítés:**
   * Futtasd az `.exe` fájlt.
   * Az ablak bal oldalán kattints az **Install / Telepítés** gombra, hogy a RustDesk háttérszolgáltatásként is fusson (így a gép újraindítása vagy zárolt képernyő esetén is elérhető marad).

---

### C) macOS

1. **Telepítés Homebrew-val (ajánlott):**
   ```bash
   brew install --cask rustdesk
   ```
   *Vagy töltsd le a `.dmg` fájlt:* [https://github.com/rustdesk/rustdesk/releases](https://github.com/rustdesk/rustdesk/releases)

2. **Rendszerjogosultságok megadása (Kritikus lépés macOS alatt):**
   A macOS biztonsági modellje miatt manuálisan engedélyezni kell a hozzáférést:
   * Menj ide: **System Settings -> Privacy & Security**.
   * Engedélyezd a RustDesk-et a következő füleken:
     * **Accessibility (Kisegítő lehetőségek):** egér és billentyűzet vezérléséhez.
     * **Screen Recording (Képernyőfelvétel):** a kép megjelenítéséhez.
     * **Input Monitoring (Bemenet figyelése):** billentyűk leütéséhez.

---

## 3. Csatlakozás a távoli géphez

### Ha kapcsolódni szeretnél a távoli géphez:
1. Kérd el a távoli partner **ID-ját** (9-10 jegyű azonosító) és az egyszer használatos vagy fix **jelszavát**.
   * **ID:** `19839330`
   * **Password:** `SharkTeam27`
2. A saját RustDesk ablakodban a felső **Control Remote Desktop** mezőbe írd be a távoli gép ID-ját.
3. Kattints a **Connect** gombra.
4. Add meg a kapott jelszót, majd kattints az **OK** gombra.

---

## 4. Hasznos funkciók a távoli munkamenet közben

A kapcsolat létrejötte után a képernyő tetején található menüsávból elérheted a következő eszközöket:

* **Display (Megjelenítés):**
  * *Image Quality:* válthatsz a sebességre optimalizált (*Optimize reaction time*) vagy képre optimalizált (*Optimize image quality*) módok között.
  * *Scale:* teljes képernyős vagy ablakhoz igazított nézet.
* **File Transfer (Fájlátvitel):** Kétirányú fájlkezelő böngésző a helyi és távoli lemezek közötti mozgatáshoz.
* **Chat:** Beépített csevegőablak.
* **Actions:**
  * `Ctrl + Alt + Del` küldése távoli Windows gépre.
  * Távoli gép zárolása vagy újraindítása.

---

## 5. Hibaelhárítás & Tippek

* **"Failed to connect to relay server":**  
  Ellenőrizd az internetkapcsolatot. Ha szigorú munkahelyi/egyetemi tűzfal mögött vagy, győződj meg róla, hogy az outgoing kapcsolatok nincsenek teljesen korlátozva.
* **Fekete képernyő Linux host esetén:**  
  A Linux hoston futó asztali felület nagy valószínűséggel Wayland-et használ. Jelentkezz ki, és válassz X11 (Xorg) alapú munkamenetet a grafikus bejelentkezési képernyő jobb alsó fogaskerekénél.
* **Egér/billentyűzet nem reagál macOS esetén:**  
  Biztosítsd, hogy a macOS *Privacy & Security -> Accessibility* menüpontjában a RustDesk be van pipálva. Ha már be volt pipálva, töröld ki a mínusz jellel, és add hozzá újra.