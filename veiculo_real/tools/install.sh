#!/bin/bash

echo "Instalando dependências do sistema..."
sudo apt update
sudo apt install -y python3-tk rsync openssh-client sshpass python3-pip

echo "Instalando dependências Python..."
pip install -r "$(dirname "$0")/requirements.txt"

echo "Instalação concluída."
