# -*- coding: utf-8 -*-
"""
Classical 2D Dubins Curve
"""
from collections import namedtuple
from math import *

import numpy as np

import matplotlib.patches as patches
from matplotlib.path import Path

#def mod2pi(angle):
#    return angle % (2*pi)

def fmodr(x, y):
	return x - y*floor(x/y)

def mod2pi(theta):
	return fmodr(theta, 2*pi)

Dubins = namedtuple("Dubins", "t p q length case")

########################################
########################################
class DubinsCurve(object):

	qi = 0
	qf = 0
	rhomin = 0
		
	t = 0
	p = 0
	q = 0
	length = 0
	case = ''
	
	_paths = None
	_minPath = ''
	
	########################################
	""" Public Methods """
	def __init__(self, qi, qf, rhomin=1):
		
		self.qi = qi
		self.qf = qf
		self.rhomin = rhomin
		
		self._paths = []
		
		self._initialize()
	
	########################################
	def __str__(self):
		
		return str(self._minPath)

	########################################
	def getCoordinatesAt(self, offset):

		# Offset normalizado
		noffset = offset/self.rhomin       

		# Translação para a origem
		qi = [0, 0, self.qi[2]]        

		# Gerando as configurações intermediárias            
		l1 = self.t
		l2 = self.p
		q1 = self.getPositionInSegment(l1, qi, self.case[0]) # Final do segmento 1
		q2 = self.getPositionInSegment(l2, q1, self.case[1]) # Final do segmento 2

		# Obtendo o restante das configurações
		if (noffset < l1):
			q = self.getPositionInSegment(noffset, qi, self.case[0])
		elif (noffset < (l1+l2)):
			q = self.getPositionInSegment(noffset-l1, q1, self.case[1])
		else:
			q = self.getPositionInSegment(noffset-l1-l2, q2, self.case[2])        

		# Translação para a posição anterior
		q[0] = q[0] * self.rhomin + self.qi[0]
		q[1] = q[1] * self.rhomin + self.qi[1]
		q[2] = mod2pi(q[2])         
		
		return q
	
	########################################
	def getPositionInSegment(self, offset, qi, case):
		
		q = [0, 0, 0]
		if (case == 'L'):
			q[0] = qi[0] + sin(qi[2]+offset) - sin(qi[2])
			q[1] = qi[1] - cos(qi[2]+offset) + cos(qi[2])
			q[2] = qi[2] + offset
		elif (case == 'R'):
			q[0] = qi[0] - sin(qi[2]-offset) + sin(qi[2])
			q[1] = qi[1] + cos(qi[2]-offset) - cos(qi[2])
			q[2] = qi[2] - offset
		elif (case == 'S'):
			q[0] = qi[0] + cos(qi[2]) * offset
			q[1] = qi[1] + sin(qi[2]) * offset
			q[2] = qi[2]; 
			
		return q            

	########################################
	def getSamplingPoints(self, res=0.1):
		
		points = []    
		range = np.linspace(0.0, self.length, int(self.length/res))
		for offset in range:
			q = self.getCoordinatesAt(offset)
			#plt.scatter(q[0], q[1])
			points.append(q)  
			
		return np.array(points)       
		
	########################################
	def draw(self, ax, drawHeadings=False, drawCurve=True):
		
		points = self.getSamplingPoints()
		
		if drawCurve:
			x = points[:, 0]
			y = points[:, 1]
		else:
			x = np.take(points[:, 0], [1,-1])
			y = np.take(points[:, 1], [1,-1])
		
		ax.plot(x, y, linewidth=2, c='b', zorder=10)
		
		if drawHeadings:
			self._drawTriangle([self.qi[0], self.qi[1]], self.qi[2], 1, [.95, .95, 0], ax)
			self._drawTriangle([self.qf[0], self.qf[1]], self.qf[2], 1, [.95, .95, 0], ax)
	
	########################################
	""" Private Methods """
	def _initialize(self):

		dx = self.qf[0] - self.qi[0];
		dy = self.qf[1] - self.qi[1];
		D = sqrt(dx**2 + dy**2);

		# Distância normalizada
		d = D/self.rhomin;          
	
		#É necessário converter as posições
		theta = mod2pi(atan2(dy, dx));
		a = mod2pi(self.qi[2] - theta);
		b = mod2pi(self.qf[2] - theta);
		
		# CSC
		pathLSL = self._LSL(a, b, d)
		pathRSR = self._RSR(a, b, d)
		pathLSR = self._LSR(a, b, d)
		pathRSL = self._RSL(a, b, d)     
	
		# CCC
		pathRLR = self._RLR(a, b, d)
		pathLRL = self._LRL(a, b, d)
	
		# Escolhendo o menor caminho
		self._paths = [pathLSL, pathRSR, pathLSR, pathRSL, pathRLR, pathLRL]
		#print(self._paths)
		self._minPath = self._paths[0]        
		for p in self._paths:
			if p.length < self._minPath.length:
				self._minPath = p
			   
		self.t = self._minPath.t
		self.p = self._minPath.p
		self.q = self._minPath.q
		self.length = self._minPath.length
		self.case = self._minPath.case
		

	""" CSC Paths """
	########## LSL ##########
	def _LSL(self, a, b, d):
		
		p2 = 2 + d**2 - 2*cos(a-b) + 2*d*(sin(a)-sin(b))
	
		if (p2 > 0):
			aux = atan2(cos(b)-cos(a), d+sin(a)-sin(b))
			t = mod2pi(-a + aux)
			p = sqrt(p2)
			q = mod2pi(b - aux)
		else:
			t = float('inf')
			p = float('inf')    
			q = float('inf') 

		length = (t+p+q) * self.rhomin        
		case = 'LSL'
		
		#print(path)
		return Dubins(t, p, q, length, case)


	########## RSR ##########
	def _RSR(self, a, b, d):
		
		p2 = 2 + d**2 - 2*cos(a-b) + 2*d*(sin(b)-sin(a))         

		if (p2 > 0):    
			aux = atan2(cos(a)-cos(b), d-sin(a)+sin(b))
			t = mod2pi(a - aux)
			p = sqrt(p2)
			q = mod2pi(mod2pi(-b) + aux)
		else:
		   t = float('inf')
		   p = float('inf')
		   q = float('inf') 
		
		length = (t+p+q) * self.rhomin        
		case = 'RSR'
		
		#print(path)
		return Dubins(t, p, q, length, case)
		
	
	########## LSR ##########
	def _LSR(self, a, b, d):
		
		p2 = -2 + d**2 + 2*cos(a-b) + 2*d*(sin(a)+sin(b))
				
		if (p2 > 0):        
			p = sqrt(p2)

			aux = atan2(-cos(a)-cos(b), d+sin(a)+sin(b)) - atan(-2/p)

			t = mod2pi(-a + aux)
			q = mod2pi(-mod2pi(b) + aux)
			
		else:         
			t = float('inf')
			p = float('inf')
			q = float('inf')

		length = (t+p+q) * self.rhomin
		case = 'LSR'
		
		#print(path)
		return Dubins(t, p, q, length, case)
		
	   
	########## RSL ##########
	def _RSL(self, a, b, d):
		
		p2 = d**2 - 2 + 2*cos(a-b) - 2*d*(sin(a)+sin(b))
					  
		if (p2 > 0):       
			p = sqrt(p2)

			aux = atan2(cos(a)+cos(b), d-sin(a)-sin(b)) - atan(2/p)

			t = mod2pi(a - aux)
			q = mod2pi(b - aux) 

		else:           
			t = float('inf')
			p = float('inf')
			q = float('inf')
						
		length = (t+p+q) * self.rhomin
		case = 'RSL';

		#print(path)
		return Dubins(t, p, q, length, case)
	   

	""" CCC Paths """
	
	########## RLR ##########
	def _RLR(self, a, b, d):
		
		aux = (6 - d**2 + 2*cos(a-b) + 2*d*(sin(a)-sin(b)))/8;

		if (abs(aux) <= 1):           
			p = mod2pi(-acos(aux));   
			t = mod2pi(a - atan2(cos(a)-cos(b), d-sin(a)+sin(b)) + p/2)
			q = mod2pi(a - b - t + p)

		else:           
			t = float('inf')
			p = float('inf')
			q = float('inf')
			
		length = (t+p+q) * self.rhomin  
		case = 'RLR'
		
		#print(path)
		return Dubins(t, p, q, length, case)
		

	########## LRL ##########
	def _LRL(self, a, b, d):
		
		aux = (6 - d**2 + 2*cos(a-b) + 2*d*(-sin(a)+sin(b)))/8;
		
		if (abs(aux) <= 1):        
			p = mod2pi(-acos(aux))
			t = mod2pi(-a + atan2(-cos(a)+cos(b), d+sin(a)-sin(b)) + p/2)
			q = mod2pi(b - a - t + p)
		
		else:        
			t = float('inf')
			p = float('inf')
			q = float('inf')
			
		length = (t+p+q) * self.rhomin      
		case = 'LRL'
		
		#print(path)
		return Dubins(t, p, q, length, case)
	
	######### DRAWING ###########    
	def _drawTriangle(self, position, orientation, scale, color, axis):   
	
		x, y = position     
		th = orientation
				
		# Matriz de rotacao
		Rz = np.matrix([[cos(th), -sin(th)], [sin(th), cos(th)]])        
		R = Rz    
		
		# Aplica transformacoes
		tip_i = scale*np.array([[0.4], [0]])
		tail1_i = scale*np.array([[-0.25], [0.25]])
		tail2_i = scale*np.array([[-0.25], [-0.25]])
		tailm_i = scale*np.array([[-0.15], [0]])
	
		tip_f = R*tip_i + np.array([[x], [y]])
		tail1_f = R*tail1_i + np.array([[x], [y]])
		tail2_f = R*tail2_i + np.array([[x], [y]])
		tailm_f = R*tailm_i + np.array([[x], [y]])
	
		verts = np.concatenate((tip_f.T, tail1_f.T, tailm_f.T, tail2_f.T, tip_f.T), axis=0)

		patch = patches.PathPatch(Path(verts[:,0:2]), facecolor=color, ec='k', zorder=15)
		axis.add_patch(patch)
