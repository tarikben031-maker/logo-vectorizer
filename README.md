# Vectoriseur de logos

Site + API qui transforme un logo PNG/JPG en SVG propre :
angles nets, lignes droites, cercles parfaits, courbes lisses.
100 % algorithmique : pas de clé API, pas de coût par image.

## Deux versions

- **`docs/` — version navigateur (recommandée)** : tout tourne dans le navigateur du visiteur
  (JavaScript + Web Worker). Aucun serveur, aucune limite, gratuit, images jamais envoyées.
  **Multicolore** : détecte automatiquement les couleurs (k-means, jusqu'à 20, dégradés compris)
  ou un nombre choisi ; une couche par couleur, empilées sans jour entre les couleurs ; chaque
  couleur est modifiable avant le téléchargement. Les dégradés deviennent des aplats (postérisation).
  Hébergement : **GitHub Pages**.
- **Racine — version serveur** (FastAPI, pour Vercel) : même algorithme en Python, avec une API HTTP.

## Mettre en ligne la version navigateur (GitHub Pages)

1. Sur GitHub, ouvrez le dépôt → **Settings** → **Pages** (menu de gauche).
2. **Source** : *Deploy from a branch*. **Branch** : `main`, dossier **`/docs`** → **Save**.
3. Après 1 à 2 minutes, le site est en ligne sur `https://<votre-compte>.github.io/logo-vectorizer/`.

Fichiers : `docs/index.html` (interface), `docs/engine.js` (moteur), `docs/worker.js` (calcul en arrière-plan), `docs/bgremove.js` (suppression du fond : uni ou damier « faux transparent », avec ou sans vectorisation).

## Contenu de la version serveur

| Fichier | Rôle |
|---|---|
| `app.py` | API FastAPI (`/api/vectorize`, `/api/health`) |
| `vectorizer.py` | Le moteur de vectorisation |
| `public/index.html` | Le site (upload, aperçu comparé, couleurs, téléchargement SVG/PNG) |
| `requirements.txt` | Dépendances Python (FastAPI, NumPy, OpenCV, Pillow) |
| `vercel.json` | Durée max de la fonction (120 s) |
| `.python-version` | Python 3.12 |

## Mettre en ligne sur Vercel (gratuit, sans terminal)

1. **GitHub** : créez un compte sur github.com → bouton **New repository** → nom `logo-vectorizer` → **Create**.
2. Sur la page du dépôt : **uploading an existing file** → glissez **tout le contenu** de ce dossier
   (y compris le dossier `public`) → **Commit changes**.
3. **Vercel** : allez sur vercel.com → **Sign up** avec votre compte GitHub (plan **Hobby**, gratuit).
4. **Add New… → Project** → choisissez `logo-vectorizer` → **Import** → **Deploy**.
   Vercel détecte FastAPI tout seul. Le premier déploiement prend 1 à 3 minutes.
5. Votre site est en ligne sur `https://logo-vectorizer-xxxx.vercel.app`.

Chaque modification envoyée sur GitHub redéploie le site automatiquement.

### Variante avec le terminal

```bash
npm i -g vercel
cd logo-vectorizer
vercel          # première fois : connexion + création du projet
vercel --prod   # mise en production
```

## Utiliser l'API

```bash
# SVG directement
curl -X POST --data-binary @logo.png -H "Content-Type: image/png" \
  "https://VOTRE-SITE.vercel.app/api/vectorize?format=svg" -o logo.svg

# SVG sans fond
curl -X POST --data-binary @logo.png \
  "https://VOTRE-SITE.vercel.app/api/vectorize?format=svg&transparent=1" -o logo.svg

# JSON : svg, svg_transparent, width, height, fg, bg, shapes, arcs, ms
curl -X POST --data-binary @logo.png "https://VOTRE-SITE.vercel.app/api/vectorize"
```

## Limites

- Version serveur (Python) : logos à **2 couleurs** (fond + une couleur). La version navigateur gère jusqu'à 20 couleurs.
- Image max **4,4 Mo** (limite Vercel) ; le site réduit automatiquement les images plus lourdes.
  Les images de plus de 3000 px sont réduites à 3000 px avant la vectorisation.
- Temps de calcul : 2 à 15 s selon la taille du logo.
- Plan **Hobby** de Vercel : **usage personnel / non commercial uniquement**, et 4 h de CPU actif
  par mois incluses (environ 1 000 à 2 000 logos). Pour un service payant, passez au plan Pro.
