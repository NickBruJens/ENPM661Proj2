import time
import numpy as np

initial_state = tuple(input('input board state in the form (1,2,3),(4,5,6),(7,8,0) : '))
start_time = time.time()
goal_state = ((1,2,3),(4,5,6),(7,8,0))
moves_state = {"0": initial_state}  # dictionary that pairs possible moves to game states

def check_solvable(game_state):  # checks if the game state is solvable
    inversions = []
    game_state = list(np.array(game_state).flatten())  # flatten the 2d list
    game_state.remove(0)  # remove 0 from the list
    for i in range(len(game_state)):
        boolean_ary = (game_state[i] > game_state[i:])  # is current value greater then following values
        inversion_indices = np.where(boolean_ary == True)  # indices where this is true
        inversions.append(len(inversion_indices[0]))  # add these inversions to the list of inversions
    num_inversions = np.sum(np.array(inversions))  # sum all the inversions
    print('There are ' + str(num_inversions) + ' inversions')
    if num_inversions % 2 != 0:
        print('Game is unsolvable!')
        return False
    else:
        return True

def effect_of_move(state, move):  # returns the state of the game after a move has been preformed

    state = np.array(state)   # convert tuple to numpy array
    index0 = np.where(state == 0)  # find the index of 0 "the empty tile space"

    if move == 'U':  # empty space is moved upwards
        index_move_tile = index0[0]-1, index0[1]  # index of tile above empty space
    elif move == 'D':  # empty space is moved downwards
        index_move_tile = index0[0]+1, index0[1]  # index of tile below empty space
    elif move == 'L':  # empty space is moved left
        index_move_tile = index0[0], index0[1]-1  # index of time to the left of empty space
    elif move == 'R':  # empty space is moved right
        index_move_tile = index0[0], index0[1]+1  # index of tile to the right of empty space

    state[index0] = state[index_move_tile]  # place tile into empty space
    state[index_move_tile] = 0  # create new empty space in place of the moved tile
    return tuple(map(tuple, state))  # returns new state after the move was preformed, returns a tuple

def physically_allowable_moves(state):  # returns list of allowable moves

    moves = list()
    state = np.array(state)  # converts state to numpy array
    index0 = np.where(state == 0)  # index of the empty tile

    if index0[1] < 2:  # places allowable moves into move list
        moves.append('R')
    if index0[1] > 0:
        moves.append('L')
    if index0[0] > 0:
        moves.append('U')
    if index0[0] < 2:
        moves.append('D')

    return moves  # returns list of allowable moves

def new_child_states_to_dict(state, key):  # adds all possible new child states to dictionary from given state

    moves = physically_allowable_moves(state)  # get list of allowable moves from current state
    previous_move = key[-1]  # remove previous move from available moves
    if previous_move == 'U':
        moves.remove('D')
    if previous_move == 'D':
        moves.remove('U')
    if previous_move == 'L':
        moves.remove('R')
    if previous_move == 'R':
        moves.remove('L')

    for move in moves:  # for all new possible moves
        new_state = effect_of_move(state, move)  # find the new state after each possible move
        if new_state not in moves_state.values():  # place any new states into the dictionary
            moves_state[key + move] = new_state

def output_solution(plan):  # outputs the steps needed to solve the game in a text file
    file_object = open('nodePath.txt','w')
    steps = []
    moves = str(plan[0])
    for i in range(len(moves)):
         steps.append(moves)
         moves = moves[:-1]
    steps = steps[::-1]
    i = 0
    for step in steps:
        state =  moves_state[step]
        file_object.write('--------- move: ' + str(i) + '\n')
        file_object.write("|"+str(state[0][0]) +' '+str(state[0][1])+ ' ' +str(state[0][2])+"|"+'\n')
        file_object.write("|" + str(state[1][0]) + ' ' + str(state[1][1]) + ' ' + str(state[1][2]) + "|"+'\n')
        file_object.write("|" + str(state[2][0]) + ' ' + str(state[2][1]) + ' ' + str(state[2][2]) + "|"+'\n')
        i = i+1
    file_object.close()

def output_all_states():  # outputs all game states into a text file
    file_object = open('Nodes.txt','w')
    all_keys = moves_state.keys()
    all_keys = sorted(all_keys)
    for key in all_keys:
        state = moves_state[key]
        file_object.write('--------- moves: ' + str(key) + '\n')
        file_object.write("|" + str(state[0][0]) + ' ' + str(state[0][1]) + ' ' + str(state[0][2])+"|" + '\n')
        file_object.write("|" + str(state[1][0]) + ' ' + str(state[1][1]) + ' ' + str(state[1][2]) + "|"+'\n')
        file_object.write("|" + str(state[2][0]) + ' ' + str(state[2][1]) + ' ' + str(state[2][2]) + "|"+'\n')
    file_object.close()

def output_node_info():
    file_object = open('NodesInfo.txt', 'w')
    file_object1 = open('ordered_NodesInfo.txt', 'w')
    all_keys = moves_state.keys()
    all_keys = sorted(all_keys)
    file_object.write(str(len(all_keys)) + ' states where explored to find the solution' + '\n')
    i = 0
    for key in all_keys:
        parent_index = 0
        if len(key) >=2:
            parent_key = key[:-1]
            parent_index = int((np.where(np.array(all_keys)==parent_key))[0])

        file_object.write(str(i)+' '+str(parent_index)+'\n')
        file_object1.write(str(key)+'\n')
        i = i+1
    file_object.close()


if not check_solvable(initial_state):  # exit if game is unsolvable
    exit()

while goal_state not in moves_state.values():  # continue to run until a solution is found

    keys_to_explore = [x for x in moves_state if (len(x)) == len(max(moves_state, key=len))]  # Only explore new states
    # that extend from the longest previous sequence of moves. Prevents exploration into terminal child states

    print(str(len(keys_to_explore)) + ' states will be explored : ' + 'at least ' +
          str(len(max(keys_to_explore, key=len))) + ' moves will be needed')

    for key in keys_to_explore:  # find new child states
        new_child_states_to_dict(moves_state[key], key)

plan = [plan for plan, state in moves_state.items() if state == goal_state]  # pulls the solution from the dictionary
print "--------------------------------------------------------------------"
print("Puzzle solved in " + str(time.time()-start_time) + ' seconds')
print('Here is the solution!!')

print(plan)

# create text files
output_solution(plan)
output_all_states()
output_node_info()
