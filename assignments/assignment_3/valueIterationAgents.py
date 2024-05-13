# valueIterationAgents.py
# -----------------------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

import mdp, util
import numpy as np

from learningAgents import ValueEstimationAgent

class ValueIterationAgent(ValueEstimationAgent):
	"""
	* Please read learningAgents.py before reading this.*

	A ValueIterationAgent takes a Markov decision process
	(see mdp.py) on initialization and runs value iteration
	for a given number of iterations using the supplied
	discount factor.
	"""
	def __init__(self, mdp, discount = 0.9, iterations = 100):
		"""
		Your value iteration agent should take an mdp on
		construction, run the indicated number of iterations
		and then act according to the resulting policy.

		Some useful mdp methods you will use:
		mdp.getStates()
		mdp.getPossibleActions(state)
		mdp.getTransitionStatesAndProbs(state, action)
		mdp.getReward(state, action, nextState)

		"""
		self.mdp = mdp
		self.discount = discount
		self.iterations = iterations
		self.values = util.Counter() # A Counter is a dict with default 0

		"*** YOUR CODE HERE ***"
		n_iter = 1	
		while n_iter <= self.iterations:			
			U = self.values.copy()
			for s in self.mdp.getStates():
				values_list = [sum([p*(self.mdp.getReward(s, a, s1) + self.discount*U[s1]) for (s1,p) 
										in self.mdp.getTransitionStatesAndProbs(s,a)]) for a in self.mdp.getPossibleActions(s)]
				if len(values_list) != 0:
					self.values[s] = max(values_list)							
			n_iter += 1
			
	def getValue(self, state):
		"""
		Return the value of the state (computed in __init__).
		"""
		return self.values[state]


	def getQValue(self, state, action):
		"""
		The q-value of the state action pair
		(after the indicated number of value iteration
		passes).  Note that value iteration does not
		necessarily create this quantity and you may have
		to derive it on the fly.
		"""
		"*** YOUR CODE HERE ***"
		q_value = sum([p*(self.mdp.getReward(state, action, s1) + self.discount*self.values[s1]) for (s1,p) in self.mdp.getTransitionStatesAndProbs(state,action)])
		return q_value
		util.raiseNotDefined()

	def getPolicy(self, state):
		"""
		The policy is the best action in the given state
		according to the values computed by value iteration.
		You may break ties any way you see fit.  Note that if
		there are no legal actions, which is the case at the
		terminal state, you should return None.
		"""
		"*** YOUR CODE HERE ***"		
		if len(self.mdp.getPossibleActions(state)) == 0:
			return None
		else:
			actions_list = self.mdp.getPossibleActions(state)
			index = np.argmax([sum([p*self.values[s1] for (s1,p) in self.mdp.getTransitionStatesAndProbs(state,a)]) for a in actions_list])
		return actions_list[index]		
		util.raiseNotDefined()

	def getAction(self, state):
		"Returns the policy at the state (no exploration)."
		return self.getPolicy(state)
  
