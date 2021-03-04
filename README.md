# IAProject

**Intelligent agent** capable of playing the **Sokoban** game.
<p align="left">
    <img src="images/playerFace.png">
</p>

## Course
This project was developed under the [Artifical Intelligence](https://www.ua.pt/en/uc/12287) course of [University of Aveiro](https://www.ua.pt/).

## Search Algorithm
* We implemented the **A\*** path-finder algorithm.
* This algorithm is proven to expand the **minimal** number of paths when using the **same** heuristic.
* This algorithm is used to calculate the pushes. To find the path from the keeper to the box of each push, we use a **Breadth-first search** algorithm.

## Heuristic
We implemented the **greedy** heuristic to sort the nodes in the queue. It is a fast heuristic to calculate and reduces the search time by quite a bit.

## Dealing with deadlocks
This agent is able to deal with the following types of deadlocks:
* **Simple** deadlocks
* **Freeze** deadlocks

## Install
Make sure you are running Python 3.5 or higher

1. Create a virtual environment (venv)
```bash
python3 -m venv venv
```

2. Activate the virtual environment (you need to repeat this step, and this step only, every time you start a new terminal/session):
```bash
source venv/bin/activate
```

3. Install the game requirements:
```bash
pip install -r requirements.txt
```

## How to run/play
open 3 terminals:

`$ python3 server.py`

`$ python3 viewer.py`

`$ python3 client.py`

### Keys
Directions: arrows

## Authors
* **Eduardo Santos**: [eduardosantoshf](https://github.com/eduardosantoshf)
* **Pedro Bastos**: [bastos-01](https://github.com/bastos-01)

