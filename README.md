# ADCS CubeSat Project

Questo repository contiene un progetto dimostrativo di **Attitude Determination and Control System (ADCS)** per un CubeSat. L'obiettivo è fornire una base didattica e pratica da mostrare a potenziali aziende nei settori **aerospazio e difesa**, evidenziando competenze in modellazione dinamica, controllo e sensor fusion.

---

## Contenuti
- **`adcs_simulation.py`** – Simulazione dinamica di un CubeSat soggetto a coppie esterne, con controllore PD.
- **`ekf_estimation.py`** – Implementazione di un filtro di Kalman esteso (EKF) per stimare l'assetto da misure rumorose di sensori (giroscopi, magnetometri, sensori solari).
- **`plots/`** – Grafici dei risultati (angoli di assetto, errori di stima, input di controllo).
- **`README.md`** – Documentazione completa del progetto, istruzioni e background teorico.

---

## Requisiti
- Python 3.9+
- Librerie: `numpy`, `scipy`, `matplotlib`

Installazione rapida:
```bash
pip install numpy scipy matplotlib
```

---

## Esempio di utilizzo
### 1. Simulazione dinamica e controllo
```bash
python adcs_simulation.py
```
Genera la dinamica del CubeSat con disturbi esterni e un controllore PD per la stabilizzazione.

### 2. Stima dell'assetto con EKF
```bash
python ekf_estimation.py
```
Applica un EKF per stimare l'assetto a partire da sensori rumorosi.

---

## Struttura del progetto

1. **Modello dinamico del CubeSat** – equazioni di Eulero per la dinamica rotazionale.
2. **Controllo** – legge di controllo PD per stabilizzazione dell'assetto.
3. **Sensori simulati**:
   - Giroscopio (con bias e rumore gaussiano)
   - Magnetometro
   - Sensore solare (selezione visibilità)
4. **Filtro di Kalman esteso (EKF)** – fusione dei dati per stimare assetto e velocità angolari.

---

## Risultati attesi
- Stabilizzazione del CubeSat verso un assetto desiderato.
- Confronto tra assetto reale e stimato.
- Analisi dell'errore di stima con diversi livelli di rumore.

---

## Prossimi sviluppi
- Implementazione di controllori avanzati (LQR, Sliding Mode).
- Aggiunta di perturbazioni realistiche (gravità, aerodinamica, radiazione solare).
- Simulazione completa di missione CubeSat.

---

## Motivazione del progetto
Questo progetto dimostra competenze chiave richieste in aziende di **aerospazio e difesa**:
- Modellazione matematica di sistemi dinamici non lineari.
- Implementazione di algoritmi di controllo real-time.
- Tecniche di **sensor fusion** e filtraggio stocastico (EKF).
- Abilità di documentare e presentare codice in maniera professionale.

