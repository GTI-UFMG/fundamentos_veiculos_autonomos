#!/usr/bin/env python3
# rsync_gui_with_default_pass.py
# GUI Tkinter para envio de arquivos e execução remota em múltiplas Raspberry Pis,
# com senha SSH padrão (DEFAULT_PASS) pré-preenchida no campo.

import os
import re
import shutil
import subprocess
import threading
import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext

# =========================
# Configurações (ajuste aqui)
# =========================
SSH_USER = "alunos"
DEFAULT_PASS = "fva2023"  # <-- coloque aqui a senha padrão desejada, ex: "rasp123"
DEFAULT_DEST = "/home/alunos/Desktop/backup_FVA"
RSYNC_OPTS = "-avz --progress"

MACS_CARS = {
	'verde':    '2c:cf:67:1c:29:4a',
	'vermelho': 'd8:3a:dd:f1:8a:4f',
	'roxo':     '2c:cf:67:1c:29:07'
}

COLORS = {
	'verde':    '#00cc00',
	'vermelho': '#cc0000',
	'roxo':     '#8000cc'
}

CAR_ICON = "🚗 "

# =========================
# Utilitários de rede
# =========================
def normalize_mac(mac: str) -> str:
	mac = mac.strip().lower()
	mac = re.sub(r'[^0-9a-f]', '', mac)
	if len(mac) != 12:
		raise ValueError(f"MAC inválido: {mac}")
	return ':'.join(mac[i:i+2] for i in range(0, 12, 2))

def parse_ip_neigh():
	try:
		out = subprocess.check_output(["ip", "neigh"], text=True)
	except Exception:
		return []
	entries = []
	for line in out.splitlines():
		m = re.search(r'(\d+\.\d+\.\d+\.\d+)\s+.*lladdr\s+([0-9a-f:]{17})', line.lower())
		if m:
			entries.append((m.group(1), m.group(2)))
	return entries

def find_ip_by_mac_arptable(target_mac: str):
	try:
		target_mac = normalize_mac(target_mac)
	except Exception:
		return None
	for ip, mac in parse_ip_neigh():
		if mac == target_mac:
			return ip
	try:
		out = subprocess.check_output(["arp", "-n"], text=True)
	except Exception:
		out = ""
	for line in out.splitlines():
		m = re.search(r'(\d+\.\d+\.\d+\.\d+)\s+([0-9a-f:]{17})', line.lower())
		if m and normalize_mac(m.group(2)) == target_mac:
			return m.group(1)
	return None

# =========================
# Interface Principal
# =========================
class RsyncGUI(tk.Tk):
	def __init__(self):
		super().__init__()
		self.title("Gerenciador de Raspberries (envio e comandos)")
		self.geometry("950x640")
		self.devices = {}
		self.selected_files = []
		self._build_ui()
		# preenche a senha padrão (se houver)
		if DEFAULT_PASS:
			self.pass_entry.insert(0, DEFAULT_PASS)
		self.refresh_ips()

	# ----------------------------
	def _build_ui(self):
		notebook = ttk.Notebook(self)
		notebook.pack(fill="both", expand=True)

		self.tab_files = ttk.Frame(notebook)
		self.tab_cmds = ttk.Frame(notebook)
		notebook.add(self.tab_files, text="📂 Enviar Arquivos")
		notebook.add(self.tab_cmds, text="💻 Executar Comandos")

		self._build_tab_files(self.tab_files)
		self._build_tab_cmds(self.tab_cmds)

	# ----------------------------
	def _build_tab_files(self, parent):
		top = ttk.Frame(parent)
		top.pack(fill="x", padx=10, pady=8)
		ttk.Label(top, text="Dispositivos detectados (cor quando encontrados):").pack(anchor="w")
		self.devices_frame = ttk.Frame(top)
		self.devices_frame.pack(fill="x", padx=4, pady=6)

		# Linhas de dispositivos (cor apenas quando IP é encontrado)
		for i, (name, mac) in enumerate(MACS_CARS.items()):
			var = tk.IntVar(value=0)
			cb = ttk.Checkbutton(self.devices_frame, text=f"{CAR_ICON}{name.upper()} — {mac}",
								 variable=var, style="Default.TCheckbutton")
			cb.grid(row=i, column=0, sticky="w", padx=4, pady=2)
			ip_label = tk.Label(self.devices_frame, text="... buscando ...", width=22,
								fg="white", font=("Arial", 10, "bold"))
			ip_label.grid(row=i, column=1, sticky="w", pady=2)
			self.devices[name] = {"mac": mac, "ip_label": ip_label, "cb": cb, "var": var, "ip": None}

		# Botões de ação
		btns = ttk.Frame(parent)
		btns.pack(fill="x", padx=6, pady=6)
		ttk.Button(btns, text="Atualizar IPs", command=self.refresh_ips).pack(side="left", padx=4)
		ttk.Button(btns, text="Selecionar arquivos…", command=self.select_files).pack(side="left", padx=4)
		ttk.Button(btns, text="Adicionar pasta…", command=self.add_directory).pack(side="left", padx=4)
		ttk.Button(btns, text="Remover selecionado(s)", command=self.remove_selected).pack(side="left", padx=4)
		ttk.Button(btns, text="Limpar seleção", command=self.clear_files).pack(side="left", padx=4)

		# Lista de arquivos/pastas
		ttk.Label(parent, text="Arquivos/Pastas selecionados:").pack(anchor="w", padx=10)
		self.files_listbox = tk.Listbox(parent, height=6, selectmode="extended")
		self.files_listbox.pack(fill="x", padx=10, pady=(2, 6))

		# Destino remoto (USADO TAMBÉM NA ABA DE COMANDOS)
		dest_frame = ttk.Frame(parent)
		dest_frame.pack(fill="x", padx=10, pady=6)
		ttk.Label(dest_frame, text="Destino na Raspberry (também será o diretório de trabalho dos comandos):").pack(side="left")
		self.dest_entry = ttk.Entry(dest_frame)
		self.dest_entry.insert(0, DEFAULT_DEST)
		self.dest_entry.pack(side="left", fill="x", expand=True, padx=8)

		# SSH e opções
		opt = ttk.Frame(parent)
		opt.pack(fill="x", padx=10, pady=6)
		ttk.Label(opt, text="Usuário:").pack(side="left")
		self.user_entry = ttk.Entry(opt, width=14)
		self.user_entry.insert(0, SSH_USER)
		self.user_entry.pack(side="left", padx=4)
		ttk.Label(opt, text="Senha:").pack(side="left", padx=(8, 0))
		self.pass_entry = ttk.Entry(opt, width=14, show="*")
		self.pass_entry.pack(side="left", padx=4)
		ttk.Label(opt, text="Opções rsync:").pack(side="left", padx=(8, 0))
		self.rsync_entry = ttk.Entry(opt, width=36)
		self.rsync_entry.insert(0, RSYNC_OPTS)
		self.rsync_entry.pack(side="left", padx=4)

		# Envio
		send = ttk.Frame(parent)
		send.pack(fill="x", padx=10, pady=6)
		ttk.Button(send, text="Enviar para selecionados", command=self.send_to_selected).pack(side="left", padx=4)
		ttk.Button(send, text="Enviar para todos (com IP)", command=self.send_to_all).pack(side="left", padx=4)

		# Log
		log_frame = ttk.Frame(parent)
		log_frame.pack(fill="both", expand=True, padx=10, pady=8)
		ttk.Label(log_frame, text="Log:").pack(anchor="w")
		self.log = scrolledtext.ScrolledText(log_frame, height=10)
		self.log.pack(fill="both", expand=True)
		self.log.configure(state="disabled")

		# estilos
		style = ttk.Style()
		style.configure("Default.TCheckbutton", foreground="white", font=("Arial", 10))
		for n, c in COLORS.items():
			style.configure(f"{n}.TCheckbutton", foreground=c, font=("Arial", 10, "bold"))

	# ----------------------------
	def _build_tab_cmds(self, parent):
		ttk.Label(parent, text="Executar comandos remotos nas Raspberries selecionadas").pack(anchor="w", padx=10, pady=(10, 4))

		hint = ttk.Label(parent, text="Obs.: os comandos serão executados dentro de: (aba Enviar Arquivos) → campo 'Destino na Raspberry'",
						 foreground="#888")
		hint.pack(anchor="w", padx=10, pady=(0, 6))

		cmds_frame = ttk.Frame(parent)
		cmds_frame.pack(fill="x", padx=10, pady=4)
		ttk.Label(cmds_frame, text="Comandos (1 por linha):").pack(anchor="w")
		self.cmd_text = scrolledtext.ScrolledText(cmds_frame, height=6)
		self.cmd_text.insert("end", "pkill python3\npython3 main.py\n")
		self.cmd_text.pack(fill="x", pady=4)

		ttk.Button(parent, text="Executar nos selecionados", command=self.run_cmds_on_selected).pack(pady=6)

		self.cmd_log = scrolledtext.ScrolledText(parent, height=16)
		self.cmd_log.pack(fill="both", expand=True, padx=10, pady=6)
		self.cmd_log.configure(state="disabled")

	# ----------------------------
	# Funções utilitárias comuns
	# ----------------------------
	def log_write(self, text):
		self.log.configure(state="normal")
		self.log.insert("end", text + "\n")
		self.log.see("end")
		self.log.configure(state="disabled")

	def cmdlog_write(self, text):
		self.cmd_log.configure(state="normal")
		self.cmd_log.insert("end", text + "\n")
		self.cmd_log.see("end")
		self.cmd_log.configure(state="disabled")

	def select_files(self):
		files = filedialog.askopenfilenames(title="Selecione arquivos (Ctrl/Shift para múltiplos)")
		for f in files:
			if f not in self.selected_files:
				self.selected_files.append(f)
				self.files_listbox.insert("end", f)

	def add_directory(self):
		d = filedialog.askdirectory(title="Selecione uma pasta")
		if d:
			path = os.path.join(d, "")
			if path not in self.selected_files:
				self.selected_files.append(path)
				self.files_listbox.insert("end", path)

	def remove_selected(self):
		sel = list(self.files_listbox.curselection())
		for idx in reversed(sel):
			val = self.files_listbox.get(idx)
			self.files_listbox.delete(idx)
			try:
				self.selected_files.remove(val)
			except ValueError:
				pass

	def clear_files(self):
		self.files_listbox.delete(0, "end")
		self.selected_files = []

	def refresh_ips(self):
		"""
		Atualiza IPs; antes de consultar a ARP, tenta gerar tráfego (ping) para
		“acordar” as raspberries.
		"""
		self.log_write("🔍 Atualizando IPs ...")

		# nomes que vamos tentar pingar (pode ajustar)
		possible_hosts = ["raspberrypi.local", "raspberrypi"]

		for name, info in self.devices.items():
			# 1) tentar pingar algo antes de olhar a ARP
			#    - se já tínhamos IP salvo, pinga esse IP
			#    - senão pinga os hostnames padrão
			warmed = False
			if info.get("ip"):
				try:
					subprocess.run(
						["ping", "-c", "1", "-W", "1", info["ip"]],
						stdout=subprocess.DEVNULL,
						stderr=subprocess.DEVNULL,
						check=False,
					)
					warmed = True
				except Exception:
					pass

			if not warmed:
				for host in possible_hosts:
					try:
						subprocess.run(
							["ping", "-c", "1", "-W", "1", host],
							stdout=subprocess.DEVNULL,
							stderr=subprocess.DEVNULL,
							check=False,
						)
					except Exception:
						# se não conseguir pingar esse nome, tenta o próximo
						continue

			# 2) agora sim olha a ARP/`ip neigh`
			ip = find_ip_by_mac_arptable(info["mac"])
			info["ip"] = ip

			if ip:
				info["ip_label"].config(text=ip, fg=COLORS.get(name, "#00ff00"))
				info["cb"].configure(style=f"{name}.TCheckbutton")
				if info["var"].get() == 0:
					info["var"].set(1)
			else:
				info["ip_label"].config(text="não encontrado", fg="white")
				info["cb"].configure(style="Default.TCheckbutton")

			self.log_write(f"{name}: {ip if ip else 'não encontrado'}")

		self.log_write("✅ Atualização concluída.")


	def get_selected_devices(self):
		return [(n, i["ip"]) for n, i in self.devices.items() if i["var"].get() and i["ip"]]

	# ----------------------------
	# Envio de arquivos (aba 1)
	# ----------------------------
	def send_to_selected(self):
		targets = self.get_selected_devices()
		if not targets:
			messagebox.showinfo("Nenhum alvo", "Selecione pelo menos uma Raspberry com IP.")
			return
		threading.Thread(target=self._run_rsync_for_targets, args=(targets,), daemon=True).start()

	def send_to_all(self):
		targets = [(n, i["ip"]) for n, i in self.devices.items() if i["ip"]]
		threading.Thread(target=self._run_rsync_for_targets, args=(targets,), daemon=True).start()

	def _run_rsync_for_targets(self, targets):
		dest = self.dest_entry.get().strip()
		user = self.user_entry.get().strip() or SSH_USER
		password = self.pass_entry.get().strip()
		rsync_opts = self.rsync_entry.get().strip() or RSYNC_OPTS

		# Recursivo se houver diretório
		if any(os.path.isdir(p) for p in self.selected_files):
			if "-r" not in rsync_opts and "--recursive" not in rsync_opts:
				rsync_opts += " -r"

		has_sshpass = shutil.which("sshpass") is not None

		# Verifica existência local
		missing = [p for p in self.selected_files if not os.path.exists(p)]
		if missing:
			self.log_write("❌ Itens inexistentes:")
			for m in missing:
				self.log_write("   - " + m)
			return

		for name, ip in targets:
			self.log_write("=" * 60)
			self.log_write(f"🚀 Enviando para {name.upper()} ({ip})")
			base_cmd = ["rsync"] + rsync_opts.split()
			if password and has_sshpass:
				base_cmd = ["sshpass", "-p", password, "rsync"] + rsync_opts.split()
			elif password and not has_sshpass:
				self.log_write("⚠️ Senha informada mas 'sshpass' não está instalado. Prosseguindo sem sshpass (pode pedir senha).")

			cmd = base_cmd + self.selected_files + [f"{user}@{ip}:{dest}"]
			self.log_write("Comando: " + " ".join(cmd))
			try:
				proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False)
				out = proc.stdout or ""
				if out.strip():
					self.log_write(out.strip())
				if proc.returncode == 0:
					self.log_write(f"✅ Sucesso: {name.upper()} ({ip})")
				else:
					self.log_write(f"⚠️ Erro (rc={proc.returncode}) em {name.upper()} ({ip})")
			except Exception as e:
				self.log_write(f"❌ Exceção: {e}")

		self.log_write("🏁 Todas as transferências finalizadas.")

	# ----------------------------
	# Execução remota (aba 2) - com absolutização de main.py e cd no workdir
	# ----------------------------
	def run_cmds_on_selected(self):
		targets = self.get_selected_devices()
		if not targets:
			messagebox.showinfo("Nenhum alvo", "Selecione ao menos uma Raspberry com IP.")
			return
		cmds = [c.strip() for c in self.cmd_text.get("1.0", "end").splitlines() if c.strip()]
		if not cmds:
			messagebox.showinfo("Nenhum comando", "Digite ao menos um comando.")
			return
		threading.Thread(target=self._run_remote_cmds, args=(targets, cmds), daemon=True).start()

	def _run_remote_cmds(self, targets, cmds):
		user = self.user_entry.get().strip() or SSH_USER
		password = self.pass_entry.get().strip()
		has_sshpass = shutil.which("sshpass") is not None
		remote_workdir = (self.dest_entry.get().strip() or DEFAULT_DEST).rstrip("/")

		def absolutize_python_main(cmd_line: str) -> str:
			"""
			Se o comando for 'python3 main.py' (ou python main.py), substitui o argumento
			'main.py' por '<remote_workdir>/main.py' para garantir execução do arquivo correto.
			"""
			parts = cmd_line.strip().split()
			if not parts:
				return cmd_line
			py_bins = {"python", "python3", "/usr/bin/python", "/usr/bin/python3"}
			if parts[0] in py_bins and len(parts) >= 2:
				if parts[1] == "main.py":
					parts[1] = f'{remote_workdir}/main.py'
					return " ".join(parts)
			return cmd_line

		for name, ip in targets:
			self.cmdlog_write("\n" + "=" * 60)
			self.cmdlog_write(f"💻 Executando em {name.upper()} ({ip}) - workdir: {remote_workdir}")
			for raw_cmd in cmds:
				# 1) força caminho absoluto quando for python* main.py
				cmd_line = absolutize_python_main(raw_cmd)

				# 2) envolve com cd no workdir (redundante mas garante contexto)
				wrapped = f'cd "{remote_workdir}" && {cmd_line}'

				# 3) monta comando ssh (evita usar bash -lc; envia uma linha única)
				if password and has_sshpass:
					full_cmd = ["sshpass", "-p", password, "ssh", "-o", "StrictHostKeyChecking=no", f"{user}@{ip}", wrapped]
				else:
					full_cmd = ["ssh", "-o", "StrictHostKeyChecking=no", f"{user}@{ip}", wrapped]

				self.cmdlog_write(f"$ {wrapped}")
				try:
					proc = subprocess.run(full_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
					out = (proc.stdout or "").strip()
					if out:
						self.cmdlog_write(out)
					if proc.returncode != 0:
						self.cmdlog_write(f"⚠️ Retorno {proc.returncode} para comando: {raw_cmd}")
				except Exception as e:
					self.cmdlog_write(f"❌ Erro: {e}")
			self.cmdlog_write(f"✅ Finalizado em {name.upper()}")

# =========================
# Execução
# =========================
if __name__ == "__main__":
	if shutil.which("rsync") is None:
		print("Aviso: instale rsync (sudo apt install rsync)")
	if shutil.which("ssh") is None:
		print("Aviso: instale openssh-client (sudo apt install openssh-client)")
	app = RsyncGUI()
	app.mainloop()
