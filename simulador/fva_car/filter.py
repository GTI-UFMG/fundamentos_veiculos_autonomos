# -*- coding: utf-8 -*-
########################################
# Disciplina: Topicos em Engenharia de Controle e Automacao IV (ENG075): 
# Fundamentos de Veiculos Autonomos - 2026/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automacao
# DELT - Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################
"""
filters.py — filtros de sinal com herança e API unificada

API comum:
- filter(sample: float) -> float    # processa uma amostra e retorna o valor filtrado
- reset(value: float = 0.0) -> None # reinicia o estado interno
- value: float                      # último valor filtrado

Inclui:
- MovingAverage
- AlphaFilter (ex-"AlphaBeta", suavização exponencial)
- MedianFilter
- Kalman1D
"""
from __future__ import annotations
from abc import ABC, abstractmethod
from collections import deque
from typing import Deque, Optional
import numpy as np

# =========================
# Classe-base / Interface
# =========================
class BaseFilter(ABC):
	"""Interface comum para filtros de 1 dimensão."""
	def __init__(self, initial: float = 0.0) -> None:
		self._value: float = float(initial)

	########################################
	@property
	def value(self) -> float:
		"""Último valor filtrado."""
		return self._value

	########################################
	@abstractmethod
	def filter(self, sample: float) -> float:
		"""Processa uma amostra e retorna o valor filtrado."""
		raise NotImplementedError

	########################################
	def reset(self, value: float = 0.0) -> None:
		"""Reinicia o estado interno do filtro."""
		self._value = float(value)

# =========================
# Média móvel (janela fixa)
# =========================
class MovingAverage(BaseFilter):
	"""
	Filtro de média móvel simples em janela de tamanho N.
	Mantém um buffer de N amostras e retorna a média.
	"""
	def __init__(self, n: int = 4, initial: float = 0.0) -> None:
		super().__init__(initial=initial)
		if n <= 0:
			raise ValueError("n deve ser > 0")
		self.n: int = int(n)
		self._buffer: Deque[float] = deque([float(initial)] * n, maxlen=n)

	########################################
	def filter(self, sample: float) -> float:
		self._buffer.append(float(sample))
		self._value = float(np.mean(self._buffer))
		return self._value

	########################################
	def reset(self, value: float = 0.0) -> None:
		super().reset(value)
		self._buffer = deque([float(value)] * self.n, maxlen=self.n)

# =======================================
# Filtro exponencial (alpha / low-pass 1P)
# =======================================
class AlphaFilter(BaseFilter):
	"""
	Suavização exponencial (low-pass de 1ª ordem).
	y_k = alpha*x_k + (1-alpha)*y_{k-1}, com alpha em [0, 1].
	"""
	def __init__(self, alpha: float = 0.5, initial: float = 0.0) -> None:
		super().__init__(initial=initial)
		self.alpha: float = float(np.clip(alpha, 0.0, 1.0))

	########################################
	def filter(self, sample: float) -> float:
		x = float(sample)
		a = self.alpha
		self._value = a * x + (1.0 - a) * self._value
		return self._value

# =========================
# Mediana (janela fixa)
# =========================
class MedianFilter(BaseFilter):
	"""Filtro de mediana em janela de tamanho N."""
	def __init__(self, n: int = 5, initial: float = 0.0) -> None:
		super().__init__(initial=initial)
		if n <= 0:
			raise ValueError("n deve ser > 0")
		self.n: int = int(n)
		self._buffer: Deque[float] = deque([float(initial)] * n, maxlen=n)

	########################################
	def filter(self, sample: float) -> float:
		self._buffer.append(float(sample))
		self._value = float(np.median(self._buffer))
		return self._value

	########################################
	def reset(self, value: float = 0.0) -> None:
		super().reset(value)
		self._buffer = deque([float(value)] * self.n, maxlen=self.n)
		
# =========================
# Kalman 1D (constante)
# =========================
class Kalman1D(BaseFilter):
	"""
	Filtro de Kalman 1D para medir uma grandeza escalar constante (modelo simples).
	- Q: variância do processo (quanto o estado pode variar entre passos)
	- R: variância da medição (ruído do sensor)
	"""
	def __init__(
		self,
		process_variance: float = 1e-5,
		measurement_variance: float = 1e-2,
		initial: float = 0.0,
		initial_error_variance: float = 1.0,
	) -> None:
		
		super().__init__(initial=initial)
		self.Q: float = float(process_variance)
		self.R: float = float(measurement_variance)
		self.initial_error_variance = float(initial_error_variance)
		self.P = self.initial_error_variance

	########################################
	def filter(self, sample: float) -> float:
		# Predição
		x_priori = self._value
		P_priori = self.P + self.Q

		# Atualização (medição)
		K = P_priori / (P_priori + self.R)  # ganho de Kalman
		z = float(sample)
		self._value = x_priori + K * (z - x_priori)
		self.P = (1.0 - K) * P_priori
		return self._value

	########################################
	def reset(self, value: float = 0.0, error_variance: Optional[float] = None) -> None:
		super().reset(value)

		if error_variance is None:
			self.P = self.initial_error_variance
		else:
			self.P = float(error_variance)

# ==========================================
# Fábrica opcional (útil para configurar GUI)
# ==========================================
def make_filter(name: str, **kwargs) -> BaseFilter:
	"""
	Constrói um filtro pelo nome (case-insensitive).
	Ex.: make_filter("movingaverage", n=8, initial=0.0)
	"""
	key = name.strip().lower()
	if key in {"ma", "moving", "movingaverage", "mavg"}:
		return MovingAverage(**kwargs)
	if key in {"alpha", "alphafilter", "exp", "ema"}:
		return AlphaFilter(**kwargs)
	if key in {"median", "mediana"}:
		return MedianFilter(**kwargs)
	if key in {"kalman", "kalman1d"}:
		return Kalman1D(**kwargs)
	raise ValueError(f"Filtro desconhecido: {name!r}")


__all__ = [
	"BaseFilter",
	"MovingAverage",
	"AlphaFilter",
	"MedianFilter",
	"Kalman1D",
	"make_filter",
]

########################################
# main test
########################################
if __name__ == "__main__":

	import matplotlib.pyplot as plt

	# ====================================
	# gera sinal de teste
	# ====================================
	np.random.seed(1)

	dt = 0.02
	t = np.arange(0.0, 10.0, dt)

	# sinal verdadeiro
	signal = (
		1.0
		+ 0.8 * np.sin(2.0 * np.pi * 0.3 * t)
		+ 0.3 * np.sin(2.0 * np.pi * 1.0 * t)
	)

	# ruido gaussiano
	noise = 0.35 * np.random.randn(len(t))

	# sinal medido
	measured = signal + noise

	# adiciona alguns outliers
	outlier_idx = np.random.choice(len(t), size=12, replace=False)
	measured[outlier_idx] += np.random.choice([-2.5, 2.5], size=len(outlier_idx))

	# ====================================
	# cria filtros
	# ====================================
	f_ma = MovingAverage(
		n=8,
		initial=measured[0]
	)

	f_alpha = AlphaFilter(
		alpha=0.15,
		initial=measured[0]
	)

	f_median = MedianFilter(
		n=7,
		initial=measured[0]
	)

	f_kalman = Kalman1D(
		process_variance=1e-3,
		measurement_variance=0.12,
		initial=measured[0],
		initial_error_variance=1.0
	)


	# ====================================
	# aplica filtros
	# ====================================
	y_ma = []
	y_alpha = []
	y_median = []
	y_kalman = []

	for sample in measured:
		y_ma.append(f_ma.filter(sample))
		y_alpha.append(f_alpha.filter(sample))
		y_median.append(f_median.filter(sample))
		y_kalman.append(f_kalman.filter(sample))


	# ====================================
	# converte para numpy
	# ====================================
	y_ma = np.array(y_ma)
	y_alpha = np.array(y_alpha)
	y_median = np.array(y_median)
	y_kalman = np.array(y_kalman)


	# ====================================
	# plota resultados
	# ====================================
	plt.figure(figsize=(8, 4))

	plt.plot(
		t,
		measured,
		label="Sinal ruidoso",
		alpha=0.35
	)

	plt.plot(
		t,
		signal,
		label="Sinal verdadeiro",
		linewidth=2
	)

	plt.plot(
		t,
		y_ma,
		label="Media movel"
	)

	plt.plot(
		t,
		y_alpha,
		label="Filtro alpha"
	)

	plt.plot(
		t,
		y_median,
		label="Mediana"
	)

	plt.plot(
		t,
		y_kalman,
		label="Kalman"
	)

	plt.xlabel("Tempo [s]")
	plt.ylabel("Amplitude")
	plt.title("Comparacao dos filtros")
	plt.grid(True)
	plt.legend()
	plt.tight_layout()
	plt.show()
