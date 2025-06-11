### Instalar Robotic Toolbox python

Primero revisamos python que este bien
```bash
python3 --version
```
Luego instalamos pip si no lo detecta por default

```bash
sudo apt update
sudo apt install python3-pip
pip3 install --upgrade pip
```
Instalar el toolbox

```bash
pip3 install roboticstoolbox-python spatialmath-python --break-system-packages
pip3 install matplotlib swift-sim --break-system-packages
```

Para probarlo

```bash
python3
```

```python
from roboticstoolbox import models
robot = models.DH.Puma560()
print(robot)
```
