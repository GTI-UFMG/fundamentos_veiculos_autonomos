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
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

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
		self.title("FVA - gerenciador de controle")
		self.geometry("1024x720")
		#self.attributes("-fullscreen", True)
		#self.bind("<Escape>", lambda event: self.attributes("-fullscreen", False))
		self.devices = {}
		self.selected_files = []
		self._build_ui()
		# preenche a senha padrão (se houver)
		if DEFAULT_PASS:
			self.pass_entry.insert(0, DEFAULT_PASS)
		self.after(
					200,
					lambda: threading.Thread(
						target=self.refresh_ips,
						daemon=True
					).start()
				)
		
		self.telemetry = {}
		
		# aumenta fontes
		style = ttk.Style()
		style.configure(".", font=("Arial", 14))
		style.configure("TButton", font=("Arial", 14))
		style.configure("TLabel", font=("Arial", 14))
		style.configure("TCheckbutton", font=("Arial", 14))
		style.configure("TNotebook.Tab", font=("Arial", 14, "bold"), padding=[12, 8])

	# ----------------------------
	def _build_ui(self):

		# =========================
		# Cabecalho
		# =========================
		header = ttk.Frame(self)
		header.pack(fill="x", padx=15, pady=(10, 5))

		# titulo
		title_frame = ttk.Frame(header)
		title_frame.pack(side="left")

		ttk.Label(
			title_frame,
			text="FVA — Fundamentos de Veículos Autônomos",
			font=("Arial", 18, "bold")
		).pack(anchor="w")

		ttk.Label(
			title_frame,
			text="Gerenciador dos Veículos Experimentais",
			font=("Arial", 12)
		).pack(anchor="w")

		# logo UFMG
		BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
		logo_path = os.path.join(BASE_DIR, "assets", "ufmg_logo.png")
		self.ufmg_logo = tk.PhotoImage(file=logo_path)
		# reduz a imagem pela metade
		self.ufmg_logo = self.ufmg_logo.subsample(6, 6)

		ttk.Label(
			header,
			image=self.ufmg_logo
		).pack(side="right")

		# =========================
		# Abas
		# =========================	
		notebook = ttk.Notebook(self)
		notebook.pack(fill="both", expand=True)

		self.tab_home = tk.Frame(notebook, bg="black")
		self.tab_files = ttk.Frame(notebook)
		self.tab_cmds = ttk.Frame(notebook)
		self.tab_data = ttk.Frame(notebook)

		notebook.add(self.tab_home, text="🏠 Início")
		notebook.add(self.tab_files, text="📂 Enviar Arquivos")
		notebook.add(self.tab_cmds, text="💻 Executar Comandos")
		notebook.add(self.tab_data, text="📊 Coletar Dados")

		self._build_tab_home(self.tab_home)
		self._build_tab_files(self.tab_files)
		self._build_tab_cmds(self.tab_cmds)
		self._build_tab_data(self.tab_data)

	# ----------------------------
	def ui(self, func, *args, **kwargs):
		self.after(0, lambda: func(*args, **kwargs))
    
	# ----------------------------
	def _build_tab_home(self, parent):

		BASE_DIR = os.path.dirname(
			os.path.dirname(os.path.abspath(__file__))
		)

		image_path = os.path.join(
			BASE_DIR,
			"assets",
			"wallpaper_fva.png"
		)

		# carrega a imagem
		self.home_image = tk.PhotoImage(file=image_path)

		# label com fundo preto
		label = tk.Label(
			parent,
			image=self.home_image,
			bg="black",
			borderwidth=0,
			highlightthickness=0
		)

		# centraliza horizontal e verticalmente
		label.place(
			relx=0.5,
			rely=0.5,
			anchor="center"
		)
    
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
		ttk.Button(
					btns,
					text="Atualizar IPs",
					command=lambda: threading.Thread(
						target=self.refresh_ips,
						daemon=True
					).start()
				).pack(side="left", padx=4)
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
		self.cmd_text.insert(
							"end",
							'pkill -f "/home/alunos/Desktop/backup_FVA/main.py"\n'
							'python3 main.py\n'
							)
		self.cmd_text.pack(fill="x", pady=4)

		ttk.Button(parent, text="Executar nos selecionados", command=self.run_cmds_on_selected).pack(pady=6)

		# area inferior: grafico e terminal lado a lado
		bottom = ttk.PanedWindow(parent, orient="horizontal")
		bottom.pack(
			fill="both",
			expand=True,
			padx=10,
			pady=6
		)

		# ----------------------------
		# painel do grafico
		plot_frame = ttk.Frame(bottom)

		self.fig = Figure(figsize=(6, 4), dpi=100)
		self.ax = self.fig.add_subplot(111)

		self.ax.set_xlabel("Tempo [s]")
		self.ax.set_ylabel("Velocidade [m/s]")
		self.ax.grid(True)

		self.canvas = FigureCanvasTkAgg(
			self.fig,
			master=plot_frame
		)
		self.canvas.get_tk_widget().pack(
			fill="both",
			expand=True
		)

		# ----------------------------
		# painel do terminal
		terminal_frame = ttk.Frame(bottom)

		ttk.Label(
			terminal_frame,
			text="Terminal:"
		).pack(anchor="w")

		self.cmd_log = scrolledtext.ScrolledText(
			terminal_frame
		)
		self.cmd_log.pack(
			fill="both",
			expand=True
		)
		self.cmd_log.configure(state="disabled")

		# adiciona os dois lados
		bottom.add(plot_frame, weight=1)
		bottom.add(terminal_frame, weight=1)

	# ----------------------------
	def _build_tab_data(self, parent):

		ttk.Label(
			parent,
			text="Coletar dados dos experimentos armazenados nas Raspberries"
		).pack(
			anchor="w",
			padx=10,
			pady=(10, 6)
		)

		# dispositivos
		devices_frame = ttk.Frame(parent)
		devices_frame.pack(fill="x", padx=10, pady=6)

		ttk.Label(
			devices_frame,
			text="Os carrinhos selecionados na aba 'Enviar Arquivos' serão utilizados."
		).pack(anchor="w")

		# pasta local
		local_frame = ttk.Frame(parent)
		local_frame.pack(fill="x", padx=10, pady=10)

		ttk.Label(
			local_frame,
			text="Destino no computador:"
		).pack(side="left")

		self.data_dest_entry = ttk.Entry(local_frame)
		self.data_dest_entry.insert(
			0,
			os.path.join(os.getcwd(), "experimentos")
		)
		self.data_dest_entry.pack(
			side="left",
			fill="x",
			expand=True,
			padx=8
		)

		ttk.Button(
			local_frame,
			text="Selecionar...",
			command=self.select_data_destination
		).pack(side="left")

		# botao de coleta
		ttk.Button(
			parent,
			text="📥 Coletar dados dos selecionados",
			command=self.collect_data
		).pack(
			anchor="w",
			padx=10,
			pady=6
		)

		# log
		ttk.Label(
			parent,
			text="Transferências:"
		).pack(
			anchor="w",
			padx=10,
			pady=(10, 2)
		)

		self.data_log = scrolledtext.ScrolledText(
			parent,
			height=16
		)
		self.data_log.pack(
			fill="both",
			expand=True,
			padx=10,
			pady=(0, 10)
		)
		self.data_log.configure(state="disabled")
		
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
		self.ui(self.log_write, "🔍 Atualizando IPs ...")

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
				self.ui(
					info["ip_label"].config,
					text=ip,
					fg=COLORS.get(name, "#00ff00")
				)

				self.ui(
					info["cb"].configure,
					style=f"{name}.TCheckbutton"
				)

				self.ui(
					info["var"].set,
					1
				)

			else:
				self.ui(
					info["ip_label"].config,
					text="não encontrado",
					fg="white"
				)

				self.ui(
					info["cb"].configure,
					style="Default.TCheckbutton"
				)

			self.ui(
				self.log_write,
				f"{name}: {ip if ip else 'não encontrado'}"
			)

			self.ui(
				self.log_write,
				"✅ Atualização concluída."
			)


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
			
		self.telemetry = {}
		self.after(0, self.update_plot)
		
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
			self.cmdlog_write(f"💻 Executando carro {name.upper()} ({ip}) - workdir: {remote_workdir}")
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
					'''proc = subprocess.run(full_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
					out = (proc.stdout or "").strip()
					if out:
						self.cmdlog_write(out)'''
						
					proc = subprocess.Popen(
						full_cmd,
						stdout=subprocess.PIPE,
						stderr=subprocess.STDOUT,
						text=True,
						bufsize=1
					)

					'''for line in proc.stdout:
						line = line.rstrip()

						if line:
							self.cmdlog_write(f"[{name.upper()}] {line}")'''
					for line in proc.stdout:
						line = line.rstrip()

						if not line:
							continue

						if line.startswith("DATA,"):
							try:
								_, t, v, vref = line.split(",")

								if name not in self.telemetry:
									self.telemetry[name] = {
										"t": [],
										"v": [],
										"vref": []
									}

								self.telemetry[name]["t"].append(float(t))
								self.telemetry[name]["v"].append(float(v))
								self.telemetry[name]["vref"].append(float(vref))
								self.after(0, self.update_plot)

							except ValueError:
								self.cmdlog_write(
									f"[{name.upper()}] Telemetria invalida: {line}"
								)

						else:
							self.cmdlog_write(f"[{name.upper()}] {line}")

					proc.wait()
					
					if proc.returncode != 0:
						self.cmdlog_write(f"⚠️ Retorno {proc.returncode} para comando: {raw_cmd}")
				except Exception as e:
					self.cmdlog_write(f"❌ Erro: {e}")
			self.cmdlog_write(f"✅ Carro {name.upper()} finalizado")

	# ----------------------------
	# Execução remota (aba 3)
	# ----------------------------
	# ----------------------------
	def select_data_destination(self):
		directory = filedialog.askdirectory(
			title="Selecione onde salvar os dados dos experimentos"
		)

		if directory:
			self.data_dest_entry.delete(0, "end")
			self.data_dest_entry.insert(0, directory)

	# ----------------------------
	def collect_data(self):

		targets = self.get_selected_devices()

		if not targets:
			messagebox.showinfo(
				"Nenhum alvo",
				"Selecione pelo menos uma Raspberry com IP."
			)
			return

		local_base = self.data_dest_entry.get().strip()

		if not local_base:
			messagebox.showinfo(
				"Destino inválido",
				"Selecione uma pasta para salvar os dados."
			)
			return

		os.makedirs(local_base, exist_ok=True)

		threading.Thread(
			target=self._collect_data,
			args=(targets, local_base),
			daemon=True
		).start()
	
	# ----------------------------
	def _collect_data(self, targets, local_base):

		user = self.user_entry.get().strip() or SSH_USER
		password = self.pass_entry.get().strip()
		has_sshpass = shutil.which("sshpass") is not None

		remote_workdir = (
			self.dest_entry.get().strip() or DEFAULT_DEST
		).rstrip("/")

		remote_logs = remote_workdir + "/logs/"

		for name, ip in targets:

			# pasta separada para cada carrinho
			local_dest = os.path.join(local_base, name)
			os.makedirs(local_dest, exist_ok=True)

			self.datalog_write(
				f"📥 Coletando dados de {name.upper()} ({ip})..."
			)

			if password and has_sshpass:
				cmd = [
					"sshpass", "-p", password,
					"rsync",
					"-avz",
					f"{user}@{ip}:{remote_logs}",
					local_dest + "/"
				]
			else:
				cmd = [
					"rsync",
					"-avz",
					f"{user}@{ip}:{remote_logs}",
					local_dest + "/"
				]

			try:
				proc = subprocess.run(
					cmd,
					stdout=subprocess.PIPE,
					stderr=subprocess.STDOUT,
					text=True,
					check=False
				)

				out = proc.stdout or ""

				if out.strip():
					self.datalog_write(out.strip())

				if proc.returncode == 0:
					self.datalog_write(
						f"✅ Dados de {name.upper()} coletados."
					)
				else:
					self.datalog_write(
						f"❌ Erro ao coletar dados de {name.upper()} "
						f"(rc={proc.returncode})."
					)

			except Exception as e:
				self.datalog_write(
					f"❌ Erro em {name.upper()}: {e}"
				)

		self.datalog_write("🏁 Coleta finalizada.")
		
	# ----------------------------
	def datalog_write(self, text):
		self.data_log.configure(state="normal")
		self.data_log.insert("end", text + "\n")
		self.data_log.see("end")
		self.data_log.configure(state="disabled")
		
	# =========================
	def update_plot(self):

		if not self.telemetry:
			return

		self.ax.clear()

		for name, data in self.telemetry.items():
			self.ax.plot(
				data["t"],
				data["v"],
				label=f"{name.upper()} - v"
			)

			self.ax.plot(
				data["t"],
				data["vref"],
				"--",
				label=f"{name.upper()} - vref"
			)

		self.ax.set_xlabel("Tempo [s]")
		self.ax.set_ylabel("Velocidade [m/s]")
		self.ax.grid(True)
		self.ax.legend()

		self.canvas.draw_idle()
	
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
