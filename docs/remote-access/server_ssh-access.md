# Shark-Remote Távoli Hozzáférés Útmutató (Linux Mint Szerver)

Ez az útmutató bemutatja, hogyan tudsz távolról hozzáférni a **Shark-Remote (Linux Mint)** szerverhez böngészőből, illetve natív terminálból / parancssorból Linux, Windows és macOS rendszereken a Cloudflare Zero Trust alagúton keresztül.

---

## 1. Hozzáférés böngészőből (Web Terminal)

Operációs rendszertől függetlenül, külön kliensprogram telepítése nélkül is elérheted a terminált közvetlenül a böngésződből:

1. Nyisd meg az alábbi címet:
   **[https://shark-remote.danielserver.uk](https://shark-remote.danielserver.uk)**
2. A Cloudflare Zero Trust felületén add meg a jogosult **e-mail címedet**.
3. Írd be az e-mailben kapott **egyszer használatos PIN kódot**.
4. A sikeres hitelesítés után megjelenő webes konzolban lépj be a szerveres felhasználóddal:
   * **Username:** *(a saját linuxos felhasználóneved)*
   * **Password:** *(a saját jelszavad)*
---

## 2. Hozzáférés terminálból (Natív SSH kapcsolat)

A natív terminálos SSH-hoz a gépeden szükség van a `cloudflared` segédprogramra, amely a háttérben felépíti a titkosított kapcsolatot.

---

### A) Linux (Debian / Ubuntu / Linux Mint)

#### 1. lépés: `cloudflared` telepítése
Futtasd az alábbi parancsokat a terminálban a Cloudflare hivatalos tárolójának hozzáadásához és a csomag telepítéséhez:

```bash
# Kulcstároló mappa létrehozása és GPG kulcs letöltése
sudo mkdir -p --mode=0755 /usr/share/keyrings
curl -fsSL https://pkg.cloudflare.com/cloudflare-main.gpg | sudo tee /usr/share/keyrings/cloudflare-main.gpg >/dev/null

# Tároló hozzáadása az APT forrásokhoz
echo 'deb [signed-by=/usr/share/keyrings/cloudflare-main.gpg] https://pkg.cloudflare.com/cloudflared jammy main' | sudo tee /etc/apt/sources.list.d/cloudflared.list

# Csomaglista frissítése és telepítés
sudo apt update
sudo apt install -y cloudflared
```

#### 2. lépés: SSH konfiguráció beállítása
Nyisd meg vagy hozd létre a személyes SSH konfigurációs fájlodat:

```bash
nano ~/.ssh/config
```

Illeszd be a következő konfigurációs blokkot a fájl végére:

```text
Host shark-remote.danielserver.uk
    ProxyCommand cloudflared access ssh --hostname %h
    User <felhasznalonev>
```

*Mentés: `Ctrl + O`, majd `Enter`. Kilépés: `Ctrl + X`.*  
Biztosítsd a megfelelő jogosultságokat:
```bash
chmod 600 ~/.ssh/config
```

#### 3. lépés: Hitelesítés (opcionális)
Hitelesítsd magad a böngészőben. Ennek a 4. lépés futtatásakor automatikusan meg kell nyílnia, a kézi futtatás kihagyható:
```bash
cloudflared access login https://shark-remote.danielserver.uk
```
*(A parancs megnyitja a böngészőt, ahol meg kell adnod az e-mail címedet és a kapott PIN kódot).*

#### 4. lépés: Kapcsolódás
Lépj be a terminálból:
```bash
ssh <felhasznalonev>@shark-remote.danielserver.uk
```

---

### B) Windows (PowerShell / Windows Terminal / Command Prompt)

#### 1. lépés: `cloudflared` telepítése
Nyiss egy rendszergazdai **PowerShell** vagy **Terminal** ablakot:

* **Winget használatával (ajánlott):**
  ```powershell
  winget install --id Cloudflare.cloudflared
  ```
* **Vagy közvetlen letöltéssel:**
  Töltsd le a hivatalos futtatható fájlt a Cloudflare oldaláról:
  [https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-windows-amd64.exe](https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-windows-amd64.exe)  
  Nevezd át `cloudflared.exe`-re, és helyezd el egy elérési útvonalban lévő mappába (pl. `C:\Windows\System32` vagy add hozzá a felhasználói `PATH`-hoz).

#### 2. lépés: SSH konfiguráció beállítása
Nyisd meg a felhasználói SSH konfigurációs fájlt Notepad-del:

```powershell
notepad $HOME\.ssh\config
```
*(Ha a mappa vagy fájl még nem létezik, a Notepad felajánlja a létrehozását).*

Illeszd be az alábbi blokkot:

```text
Host shark-remote.danielserver.uk
    ProxyCommand cloudflared access ssh --hostname %h
    User <felhasznalonev>
```

Mentsd el (`Ctrl + S`) és zárd be a szerkesztőt.

#### 3. lépés: Hitelesítés (opcionális)
Hitelesítsd magad a böngészőben. Ennek a 4. lépésnél automatikusan végre kell hajtódnia, a lépés kihagyható:
```powershell
cloudflared access login https://shark-remote.danielserver.uk
```
*(A felugró böngészőablakban add meg az e-mailedet és a jóváhagyó kódot).*

#### 4. lépés: Kapcsolódás
Lépj be a terminálból:
```powershell
ssh <felhasznalonev>@shark-remote.danielserver.uk
```

---

### C) macOS (Terminal / iTerm2)

#### 1. lépés: `cloudflared` telepítése
A legkényelmesebb a Homebrew csomagkezelővel:

```bash
brew install cloudflared
```

*(Homebrew hiányában letöltheted a legfrissebb macOS kiadást a Cloudflare GitHub Releases oldaláról is).*

#### 2. lépés: SSH konfiguráció beállítása
Nyisd meg a konfigurációt nano-val:

```bash
nano ~/.ssh/config
```

Illeszd be a következő sort:

```text
Host shark-remote.danielserver.uk
    ProxyCommand cloudflared access ssh --hostname %h
    User <felhasznalonev>
```
*(Megjegyzés: Intel processzoros Mac esetén a path általában `/usr/local/bin/cloudflared`, vagy simán `cloudflared access ssh --hostname %h`).*

Mentsd el (`Ctrl + O`, `Enter`, majd `Ctrl + X`), és biztosítsd a fájl jogosultságait:
```bash
chmod 600 ~/.ssh/config
```

#### 3. lépés: Hitelesítés (opcionális)
Hitelesítsd magad a böngészőben. Ennek a 4. lépés futtatásakor automatikusan végre kell hajtódnia, a lépés kihagyható:
```bash
cloudflared access login https://shark-remote.danielserver.uk
```
*(A parancs megnyitja a böngészőt, ahol meg kell adnod az e-mail címedet és a kapott kódot).*

#### 4. lépés: Kapcsolódás
Lépj be a terminálból:
```bash
ssh <felhasznalonev>@shark-remote.danielserver.uk
```

---

## 3. Hibaelhárítás & Tippek

* **Elgépelt tároló elérési út javítása Linuxon:**  
  Ügyelj arra, hogy a lista fájl a `/etc/apt/sources.list.d/cloudflared.list` útvonalra kerüljön (ne a hibás `sources.list.dist.d` mappába).
* **Token lejárata:**  
  Ha a kapcsolat elutasításra kerül vagy nem válaszol, egyszerűen futtasd újra a `cloudflared access login https://shark-remote.danielserver.uk` parancsot egy friss belépési token igényléséhez.
* **VS Code Remote SSH támogatás:**  
  A fenti `~/.ssh/config` beállítás után a Visual Studio Code *Remote - SSH* kiegészítője is azonnal működik, mindössze a `shark-remote.danielserver.uk` hostot kell kiválasztani a kapcsolatok listájából.
* **Lehet hogy ki van kapcsolva vagy nincs neten a gép?**  
  Írj Molnár Daninak discordon, messengeren vagy emailen!

---

## 4. Mi történik, ha a Linux Mint szerver ki van kapcsolva?

Ha a szervergép le van állítva vagy megszakadt az internetkapcsolata, a Cloudflare Access hitelesítés (e-mail + kód) még sikeresen végbemegy a Cloudflare felhőjében, de magához a géphez nem tud felépülni az SSH munkamenet:

* **Böngészőből megnyitva:**
  * A hitelesítést követően **HTTP 502 Bad Gateway**, **504 Gateway Timeout** vagy **Error 1033 (Argo Tunnel error)** hibaoldal jelenik meg.
  * Ez azt jelzi, hogy a Cloudflare hálózata nem éri el a Linux Minten futó `cloudflared` komponenst.

* **Terminálból / VS Code-ból csatlakozva:**
  * A kapcsolat rövid várakozás után megszakad az alábbi hibaüzenetek valamelyikével:
    ```text
    kex_exchange_identification: Connection closed by remote host
    Connection closed by UNKNOWN port 65535
    ```
    vagy:
    ```text
    ssh: connect to host shark-remote.danielserver.uk port 22: Connection timed out
    ```

* **Nézd meg a weboldalon:**  
  TODO