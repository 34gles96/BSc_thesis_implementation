# Moonlight Kliens Használati Útmutató (Sunshine Host)

Ez a dokumentum a távoli munkaállomáshoz (`Shark-remote`) történő csatlakozás lépéseit, a javasolt beállításokat, a gyorsbillentyűket és a hibaelhárítást tartalmazza.

> [!WARNING]
> **Ez a környezet trappeli az egeret, amiből csak shortcuttal lehet kiszedni**
> Gyors, BeamNG.tech futtatásához tökéletes, éles képe van, de egyszerű remote munkavégzéshez annyira nem kényelmes.

---

## 1. Kliens Telepítése

### Windows
1. Töltsd le a legfrissebb telepítőt a hivatalos oldalról: [moonlight-stream.org](https://moonlight-stream.org) (vagy a GitHub Releases felületről a `MoonlightSetup-x.x.x.exe` fájlt).
2. Futtasd a telepítőt, és kövesd a varázsló utasításait.
3. *Alternatíva (Winget):*
   ```powershell
   winget install MoonlightGameStreaming.Moonlight
   ```

### macOS
1. Töltsd le a legfrissebb telepítőt a hivatalos oldalról: [moonlight-stream.org](https://moonlight-stream.org) (vagy a GitHub Releases felületről a `(Moonlight-x.x.x.dmg)` fájlt).
2. Nyisd meg a letöltött `.dmg` fájlt, majd húzd a Moonlight ikont az Applications (Alkalmazások) mappába.
3. Első indításkor, ha a macOS biztonsági figyelmeztetést dob: nyisd meg a Rendszerbeállítások $\rightarrow$ Adatvédelem és biztonság menüt, és engedélyezd a megnyitást.
4. Alternatíva (Homebrew):
    ```bash
    brew install --cask moonlight
    ```


### Linux

- AppImage (Univerzális – Debian, Ubuntu, Linux Mint, Fedora, Arch):
    1. 1. Töltsd le a legfrissebb telepítőt a hivatalos oldalról: [moonlight-stream.org](https://moonlight-stream.org) (vagy a GitHub Releases felületről a `Moonlight-x.x.x-x86_64.AppImage` fájlt).
    2. Adj neki futtatási jogosultságot, és indítsd el:
        ```Bash
        chmod +x Moonlight-*.AppImage
        ./Moonlight-*.AppImage
        ```
    3. Ha az indításkor FUSE hiányára utaló hiba jelenik meg:
        ```Bash
        sudo apt update && sudo apt install -y libfuse2t64 || sudo apt install -y libfuse2
        ```
- Flatpak (Ajánlott asztali környezetekhez):
    ```Bash
    flatpak install flathub com.moonlight_stream.Moonlight
    flatpak run com.moonlight_stream.Moonlight
    ```

## 1. Csatlakozás és Első Párosítás (PIN)

1. **Kliens indítása:** Indítsd el a Moonlight alkalmazást.
2. **Kiszolgáló hozzáadása:**
   * **Manuális felvétel:** Kattints a jobb felső sarokban a **`+` (Add PC)** gombra:
     * Távoli elérés: `moonlight.danielserver.uk`.
3. **Párosítás jóváhagyása:**
   * Kattints a kiszolgáló ikonjára; a Moonlight egy **4 számjegyű PIN-kódot** generál.
   * Nyisd meg a hoszt gép böngészőjében a Sunshine webes felületét: `https://sunshine.danielserver.uk`.
   * Lépj a felső menüsorban a **PIN** fülre, írd be a kódot, majd kattints a **Send** gombra.
   * A Moonlight felületén a lakat eltűnik, a gép zárolása feloldódik.

---

## 2. Javasolt Kliens Beállítások (Settings)

A főmenüben a fogaskerék ikonra kattintva állítsd be az alábbi értékeket az optimális késleltetéshez és minőséghez:

| Beállítási kategória | Opció | Javasolt érték / Megjegyzés |
|---|---|---|
| **Basic Settings** | Resolution | `1920x1080` (vagy a helyi monitor natív felbontása) |
| **Basic Settings** | Frame rate | `60 FPS` (vagy `120 FPS` ha a monitor támogatja) |
| **Basic Settings** | Video bitrate | Helyi LAN: `30 - 50 Mbps`<br>Távoli WAN: `15 - 25 Mbps` |
| **Display Mode** | Display mode | `Borderless Windowed` vagy `Windowed` |
| **Input Settings** | Optimize mouse for desktop | **Bekapcsolva** (távoli asztalnál sima egérmozgást biztosít) |
| **Input Settings** | Capture system keyboard shortcuts | **always** (távoli asztalnál engedélyezi a billentyűkombinációk használatát) |


---

## 3. Elérhető Profilok

* **Desktop:** A teljes Linux Mint felület elérése. Ha a szerveren nincs bejelentkezett felhasználó, közvetlenül a **LightDM Login Screen** jelenik meg.


---

## 4. Gyorsbillentyűk (Hotkeys)

A streaming ablak aktív állapota alatt használható billentyűkombinációk:

* `Ctrl + Alt + Shift + Q` — **Kilépés a streamből** (a távoli folyamatok futva maradnak a háttérben).
* `Ctrl + Alt + Shift + Z` — **Egérkurzor felszabadítása** (ablakos módban kihozza a kurzort a stream ablakból).
* `Ctrl + Alt + Shift + F` — Váltás teljes képernyő és ablakos nézet között.
* `Ctrl + Alt + Shift + S` — **Statisztikai panel** (valós FPS, hálózati késleltetés, frame drop adatok).
* `Ctrl + Alt + Shift + M` — Helyi egérkurzor ki-/bekapcsolása.

---

## 5. Megosztott vágólap Használata (Shared Clipboard)

- itt: TODO
- rustdesk: sima `Ctrl + C` - `Ctrl + V`

---

## 6. Fájlmegosztás

- ssh-n keresztül vscode remote computer funkció használatával
- rustdesk klienssel ( 3 pötty $\rightarrow$ transfer file )
- itt: TODO
---

## 6. Gyakori Hibák és Megoldásuk

TODO