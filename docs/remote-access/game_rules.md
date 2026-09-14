# Szerverhasználati és Biztonsági Házirend

Ez a szabályzat a távoli munkaállomás (`*.danielserver.uk`, `rustdesk`) stabil, biztonságos és konfliktusmentes működését szolgálja. Minden felhasználóra kötelező érvényű az SSH, a távoli asztali (Moonlight/Sunshine) és a rendszererőforrások használata során.

---

## 1. Felhasználói Fiókok és Hozzáférés

* **Kizárólag saját fiók:** Mindig a saját nevedre szóló felhasználói fiókkal jelentkezz be SSH-n Moonlighton és RustDesken keresztül egyaránt. Más fiókjába belépni vagy annak nevében folyamatokat futtatni tilos.
* **Kijelentkezés vs. Munkamenet elhagyása:**
  * Ha végeztél a munkával vagy a játékkal, mindig **jelentkezz ki** (Logout), hogy felszabadítsd a GPU-t, az X11 display-t és a memóriát a többiek számára.
  * A Moonlight stream bezárása (`Ctrl + Alt + Shift + Q`) **nem** jelentkeztet ki automatikusan; a háttérben futó nehéz folyamatokat (pl. szimuláció, renderelés) kézzel zárd be előtte.

---

## 2. Emelt Szintű Jogosultságok (`root` / `sudo`)

* **Nincs közvetlen root bejelentkezés:** A közvetlen root login tiltva van; minden adminisztratív feladatot `sudo` előtaggal, saját felhasználóból kell végezni.
* **Körültekintő szerkesztés:** 
  * Rendszerszintű konfigurációs fájlok módosítása előtt (különösen `/etc/`, `lightdm`, `fstab`, `network`) **mindig készíts biztonsági mentést** (pl. `sudo cp config.conf config.conf.bak`).
  * Ne futtass ismeretlen bash scripteket ellenőrzés nélkül emelt joggal (`curl ... | sudo bash` szigorúan tilos).
* **Csomagkezelés:** Kerüld a rendszer `apt` adatbázisának felesleges szemetelését PPA tárolókkal és globális Python csomagokkal (`pip install --break-system-packages` tilos).

---

## 3. Fejlesztési Környezet és Munkamenetek

* **Docker az elsődleges környezet:** Minden fejlesztési környezetet, háttérszolgáltatást, adatbázist és tesztalkalmazást **Docker konténerben vagy Docker Compose segítségével** kell futtatni. A hoszt operációs rendszer kizárólag a hardverillesztőket, a közvetlen GPU-szimulációt és a display démont szolgálja ki.
* **GPU erőforrások kezelése:** 
  * CUDA/AI feladatok és grafikus szimulációk (pl. BeamNG) futtatása előtt ellenőrizd a `nvidia-smi` kimenetét, hogy fut-e másnál kritikus terhelés.
  * Konténerizált GPU feladatoknál mindig használd az NVIDIA Container Toolkitet megfelelő erőforrás-korlátokkal.
* **Erőforrás-figyelés:** Hosszan futó számítási feladatoknál ügyelj arra, hogy a CPU/RAM ne telítődjön 100%-ra, ellehetetlenítve a távoli asztali elérést.

---

## 4. Tárolók, Lemezkezelés és Külső Meghajtók

* **Külső meghajtók és csatolási pontok:**
  * **Szigorúan tilos** a szerverre csatlakoztatott, de nem mountolt külső meghajtókat felcsatolni (`mount`), formázni vagy átparticionálni.
* **Saját könyvtár tiszteletben tartása:** Minden adatot, kódot és modellt a saját `/home/<felhasznalonev>/` mappádban tárolj. A közös rendszerszintű könyvtárakba (`/`, `/opt`, `/srv`) csak indokolt esetben, globális indítóknál kerülhet adat.
* **Nagy fájlok és szemetelés:** Rendszeresen takarítsd a fel nem használt Docker image-eket és cache-t (`docker system prune`).

---

## 5. Hálózat, Portok és Sunshine Streaming

* **Portkezelés:**
  * Ne nyiss új portokat az UFW tűzfalon és a routeren anélkül, hogy egyeztetnél a többiekkel.
* **Sunshine konfiguráció védelme:**
  * A Sunshine központi beállításait (`sunshine.conf`, `apps.json`) ne módosítsd egyéni igények szerint úgy, hogy az elrontsa a globális vagy a Login Screen rögzítést (`DISPLAY=:0` felülírása tilos).
  * Új streamelhető alkalmazást körültekintően adj hozzá az `apps.json`-hoz, ügyelve a leállító parancsokra (`undo`).
* **Vágólap és biztonság:** A vágólap megosztása aktív; érzékeny jelszavakat, privát kulcsokat körültekintően másolj, és töröld a vágólap előzményeit szükség esetén.

## 6. Gyakori Hibák és Megoldásuk
 - ha hiba merül fel, szólj Molnár Daninak discordon, messengeren, emailen, telefonon, vagy személyesen

## 7. Jelszavak
Mivel elolvastad a szabályokat, kiérdemelted a jelszavadat. Ezeket ebben a táblázatban találod meg:
https://docs.google.com/spreadsheets/d/1ON_kKN5WcVB_l9Y5AQHYR0e5wDDAHElJEuSag1Sg7FA/edit?gid=0#gid=0

