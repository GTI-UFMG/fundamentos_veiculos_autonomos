#!/bin/bash

set -e

echo "Instalando dependências do sistema..."
sudo apt update
sudo apt install -y python3-tk python3-pip

echo "Instalando dependências Python..."
python3 -m pip install -r "$(dirname "$0")/requirements_gui.txt"

echo "Instalação concluída."
