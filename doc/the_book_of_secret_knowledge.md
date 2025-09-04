# S5-GRO - Bases et commandes

Références recommandées:

- [Tutoriels: _Basic_ (_CLI tools_)](http://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html): Bien comprendre les outils CLI (ça accélère le déverminage)
- [Tutoriels: _Basic_ (_Client libraries_)](http://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries.html):
  - [Utiliser `colcon`](http://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#build-the-workspace)
  - [Pub/sub Python](http://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [Concepts fondamentaux de ROS 2](http://docs.ros.org/en/jazzy/Concepts/Basic.html):
  - Nodes
  - Topics
  - Interfaces
  - Services
  - Introspection with command line tools
- [Git](https://git-scm.com/docs):
  - _Basic Snapshotting_ (commandes de base)
  - _Sharing and Updating Projects_ (synchroniser avec GitHub)
  - _Inspection and Comparison_ (visualisation des changements)
- Bash: automatiser les commandes de _build_ et d'évaluation des variables d'environnements
  - [_Cheat sheet_](https://github.com/RehanSaeed/Bash-Cheat-Sheet)
  - [Référence](https://www.gnu.org/software/bash/manual/bash.html)
  - [Tutoriels: commandes de base](https://www.w3schools.com/bash/bash_commands.php)

> _N.B._ Plus vous utiliser les commandes en CLI, plus le développement et le déverminage de vos solutions sera rapide, et moins les APP seront "lourds", alors il est fortement recommandé de maximiser les interactions avec le CLI (et de minimiser l'utilisation des GUI et les clics de souris).

Références supplémentaires:

- [colcon](https://colcon.readthedocs.io/en/released/)
- [Python (3.12.X)](https://docs.python.org/3.12/library/index.html)

## Structure d'un workspace ROS

```ascii
ros2_ws/
├── build/
├── install/
│   ├── setup.bash
│   └── ...
├── log/
└── src/
    └── racecar/
        └── racecar_<package>/
```

- `ros2_ws/`: "base d'opérations" du workspace
- `build/`: pas vraiment pertinent
- `log/`: contient les logs (utile pour déverminage)
- `install/`: contient les fichiers _setup_ (variables d'environnement, "magie" ROS, etc.)
  - `setup.bash`: à évualer AVANT de lancer un programme ROS
- `src/`: contient les _packages_ ROS à _build_
- `racecar/`: le dépôt du projet, contient les _packages_ à développer
- `racecar_<package>`: les _packages_ qui sont _built_

> _N.B._ à partir d'ici, le dossier `ros2_ws` sera nommé "ws".

## Pipeline de lancement de programme ROS

**IMPORTANT** Lorsqu'on veut faire une opération qui concerne le lancement d'un programme utilisant ROS, on exécute les commandes à partir du _ws_. Sinon, ça "chie".

Voici un exemple du _pipeline_ complet pour lancer un programme ROS:

```bash
colcon build                # 1) build les packages
source install/setup.bash   # 2) Update les var. env.
ros2 <command> ...          # 3) Lancer un programme ROS
```

Pour éviter de refaire un _build_ à chaque changement dans un fichier, exécuter:

```bash
colcon build --symlink-install
```

### _Clean build_

_N.B._ Parfois, le _build_ automatique ne fonctionne plus. Si c'est le cas, exécuter à nouveau la commande. Si ça ne fonctionne toujours pas, passer au "détergent nucléaire" (_i.e._ un _build_ de A à Z):

```bash
rm -rf build log install    # Supprime tous les dossiers générés par colcon
colcon build                # NOTE: des "WARNINGS" apparaitront: c'est normal.
```

## Précision sur le fichier _setup_

Il n'est pas toujours nécessaire de "sourcer" le fichier _setup_. Par exemple, pour des changements mineurs, on peut directement relancer le programme.

Hypothèse: lorsqu'une nouvelle variable d'environnement est introduite (e.g. _topic_ , _node_ , _service_ , etc.), on doit ré-évaluer le fichier _setup_ , voire
refaire un _build_.

> Mot de l'auteur: Je ne sais pas encore quand et pourquoi on doit refaire un _build_ et/ou un _source_ : c'est encore une "boîte noire" pour moi. Donc, si des changements ne se font pas entre deux _run_ , essayez un _source_ ou un _build_ + source_, ou un _clean build_.

## GRO520: tester une solution à la problématique

Pour faciliter et accélérer les tests de vos solutions, utiliser le _bag_ du laboratoire:

```bash
ros2 bag play -l /path/to/racecar_loop
```

## Commande ros2 run

`ros2 run` est la commande utilisée pour "spin-ner" (lire "lancer") une _node_. La syntaxe est la suivante:

```bash
ros2 run <package_name> <executable_name> # e.g. ros2 run racecar_beacon lab_poll_pos
```

- _package_name_ : le nom du _package_. On le trouve dans le fichier package.xml du _package_ , mais pour le racecar, les noms des _packages_ sont racecar_<...>
- _executable_name_ : le nom du programme. Lorsque l'environnement ROS2 du racecar est bien configuré, le CLI liste tous les noms des exécutables d'un package en appuyant sur `TAB`, par exemple:

```bash
ros2 run racecar_beacon <TAB> # Parfois, ça prend deux <TAB> pour que l'auto-complétion s'active
```

## Déverminer `No executables found`

Si une commande `ros2 run` retourne l'erreur `No executables found`, et que l'exécutable existe dans le fichier `setup.py`, c'est qu'il y a une erreur dans
la _cache_ du _build_. Pour résoudre le problème, on supprime la _cache_ , et on _rebuild_ les _packages_ :

```bash
colcon build --cmake-clear-cache --symlink-install
```

Un [_clean build_](#clean-build) fonctionne aussi.

## Commande `ros2 launch`

`ros2 launch` est utilisée pour lancer plusieurs _nodes_ en même temps selon la confguration désirée. Elle n'est pas utilisée dans GRO520, et n'est pas
matière à examen.

## Python: _socket_

Références:

- [Documentation du module _socket_](https://docs.python.org/3/library/socket.html)
- [HOWTO _socket_](HOWTO socket:
) : bonne introduction aux concepts

### Créer un socket

### TCP/IP

```py
import socket

# AF_INET: IPv4, SOCK_STREAM: TCP
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
```

### UDP/IP ( Broadcast )

```py
import socket

# SOCK_DGRAM: type du datagramme
# IPPROTO_UDP: spécifie le protocole UDP
broadcast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

# Mode "broadcast" => pas besoin de connect()
broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

### Overwrite un socket s'il n'a pas été fermé lors de la dernière exécution
broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
```

### Utiliser _bind_, _listen_, _connect_, _accept_, _close_

```py
# FILE: server.py

sock.bind( (HOST, PORT) )
sock.listen(1)

(conn, addr) = sock.accept()
...
sock.close()

# FILE: client.py

sock.connect( (HOST, PORT) )
...
sock.close()
```

### Assurer la fermeture d'un socket

```py
sock = socket.socket(...)...

try:
    ... # Exécution normale
except KeyboardInterrupt: # "catch" un CTRL + C
    pass # faire RIEN
finally: # S'exécute peu importe le branchement (normal | except)
    sock.close()
```

## Python: ROS 2

Références:

- [Documentation du module _rclpy_](https://docs.ros2.org/latest/api/rclpy/api.html) : **Très** utile pour le code de `ros_monitor.py`
- [Documentation des formats pour _(un)pack_](https://docs.python.org/3.12/library/struct.html) : Voir les sections **Byte Order, Size, and Alignment** et **Format Characters**

_N.B._ Pour enlever les _squiggly lines_ et voir les prototypes des méthodes, modifier la ligne d'importation:

```py
from struct import pack, unpack # Plutôt que 'from struct import *'
```
