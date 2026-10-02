# BeamNG.tech & ROS 2 Környezet Beállítása (ADC_27)

Útmutató a távoli gépen történő munkakörnyezet felállításához, a szimulációs pálya telepítéséhez, valamint a Docker konténer buildeléséhez és futtatásához.

---

## 1. Pálya letöltése és telepítése

Lépj be a távoli gépre távoli asztalon keresztül, majd töltsd le a pályát tartalmazó archívumot:

* **Letöltési link:** [https://drive.google.com/drive/u/0/folders/12hJ9vIuaSLynekCut-fneASQQ6hx1nMO](https://drive.google.com/drive/u/0/folders/12hJ9vIuaSLynekCut-fneASQQ6hx1nMO)

### Telepítés lépései

1. Hozd létre a célkönyvtárat, ha még nem létezik:
    ```bash
   mkdir -p ~/.local/share/BeamNG/BeamNG.tech/current/levels
    ```
2. Csomagold ki a letöltött ZIP fájlt a fenti mappába úgy, hogy az alábbi struktúrát kapd:
    ```bash
   ~/.local/share/BeamNG/BeamNG.tech/current/levels/silesia_ring/
    ```
3. Ellenőrzés: Indítsd el a BeamNG.tech szimulátort. A pályaválasztó menüben meg kell jelennie a Silesia Ring pályának.

---

## 2. Repó klónozása és Docker környezet felépítése

A konténerek és kódok ütközésének elkerülése érdekében mindenki a saját felhasználói fiókjában dolgozik.

### Repó előkészítése és Git beállítás

1. Navigálj a saját home könyvtáradba és klónozd az ADC_27 projektet. Javaslom a vscode verziókezelő menüjét használni.

2. Állítsd be a Git azonosítóidat (ha korábban nem tetted meg):
    ```bash
   git config user.name "Vezetéknév Keresztnév"
   git config user.email "email@pelda.hu"
    ```

3. Mindig a saját branch-eden dolgozz! A main-be ne commitolj!


---

### Docker konténer testreszabása és buildelése

1. Nyisd meg a Compose konfigurációt:
    ```bash
   ~/ADC_27/docker/ros2_beamng_bridge/docker-compose.yml
    ```

2. Módosítsd a container_name mezőt úgy, hogy a végén szerepeljen a felhasználóneved:
    ```bash
   container_name: ros2_beamng_bridge_<username>
    ```

3. Építsd meg és indítsd el a konténert a háttérben:
    ```
   docker compose up -d --build
    ```

---

## 3. A környezet használata

### Belépés a konténerbe
```bash
docker exec -it ros2_beamng_bridge_<username> bash
```

### Szcenárió indítása
```bash
ros2 launch /ADC_27/launch/scenario_launch/launch.py
```

### Topicok és vizualizáció ellenőrzése
* Grafikus felület: rqt
* Topic lista: ros2 topic list
* Élő adat: ros2 topic echo <topic_neve>

---

## 4. Hasznos BeamNG információk
* World Editor: F11 billentyű
* Járműmodell: Wigeon
* Szenzorok: 2 db kamera az autó tetején

---

## 5. Hibaelhárítás
* Szép markdown megjelenítés VScode-ban: CTRL + SHIFT + V
* Kérdezd az AI-t...
