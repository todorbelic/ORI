import copy
import random
from abc import *


class State(object):
    """
    Apstraktna klasa koja opisuje stanje pretrage.
    """

    @abstractmethod
    def __init__(self, board, parent=None, position=None, goal_position=None, fire_position=None):
        """
        :param board: Board (tabla)
        :param parent: roditeljsko stanje
        :param position: pozicija stanja
        :param goal_position: pozicija krajnjeg stanja
        :return:
        """
        self.board = board
        self.parent = parent  # roditeljsko stanje
        if self.parent is None:  # ako nema roditeljsko stanje, onda je ovo inicijalno stanje
            self.position = board.find_position(self.get_agent_code())  # pronadji pocetnu poziciju
            self.goal_position = board.find_position(self.get_agent_goal_code())  # pronadji krajnju poziciju
            self.fire_position = board.find_position(self.get_fire_code())
        else:  # ako ima roditeljsko stanje, samo sacuvaj vrednosti parametara
            self.position = position
            self.goal_position = goal_position
            self.fire_position = fire_position
        self.depth = parent.depth + 1 if parent is not None else 1  # povecaj dubinu/nivo pretrage
        self.picked_boxes = copy.deepcopy(parent.picked_boxes) if parent is not None else []
        self.portals = copy.deepcopy(parent.portals) if parent is not None else []

    def get_next_states(self, has_boxes, initial_state):
        new_positions = self.get_legal_positions(has_boxes, initial_state)  # dobavi moguce (legalne) sledece pozicije iz trenutne pozicije
        next_states = []
        # napravi listu mogucih sledecih stanja na osnovu mogucih sledecih pozicija
        for new_position in new_positions:
            next_state = self.__class__(self.board, self, new_position, self.goal_position, self.fire_position)
            next_states.append(next_state)
        return next_states

    @abstractmethod
    def get_agent_code(self):
        """
        Apstraktna metoda koja treba da vrati kod agenta na tabli.
        :return: str
        """
        pass

    @abstractmethod
    def get_agent_goal_code(self):
        """
        Apstraktna metoda koja treba da vrati kod agentovog cilja na tabli.
        :return: str
        """
        pass

    @abstractmethod
    def get_portal_code(self):

        pass

    @abstractmethod
    def get_fire_code(self):

        pass

    @abstractmethod
    def get_legal_positions(self):
        """
        Apstraktna metoda koja treba da vrati moguce (legalne) sledece pozicije na osnovu trenutne pozicije.
        :return: list
        """
        pass

    @abstractmethod
    def is_final_state(self):
        """
        Apstraktna metoda koja treba da vrati da li je treuntno stanje zapravo zavrsno stanje.
        :return: bool
        """
        pass

    @abstractmethod
    def unique_hash(self):
        """
        Apstraktna metoda koja treba da vrati string koji je JEDINSTVEN za ovo stanje
        (u odnosu na ostala stanja).
        :return: str
        """
        pass
    
    @abstractmethod
    def get_cost(self):
        """
        Apstraktna metoda koja treba da vrati procenu cene
        (vrednost heuristicke funkcije) za ovo stanje.
        Koristi se za vodjene pretrage.
        :return: float
        """
        pass
    
    @abstractmethod
    def get_current_cost(self):
        """
        Apstraktna metoda koja treba da vrati stvarnu trenutnu cenu za ovo stanje.
        Koristi se za vodjene pretrage.
        :return: float
        """
        pass


class RobotState(State):
    def __init__(self, board, parent=None, position=None, goal_position=None, fire_position=None):
        super(self.__class__, self).__init__(board, parent, position, goal_position, fire_position)
        # posle pozivanja super konstruktora, mogu se dodavati "custom" stvari vezani za stanje
        # TODO 6: prosiriti stanje sa informacijom da li je robot pokupio kutiju
        if parent is not None:
            self.picked_boxes = copy.deepcopy(parent.picked_boxes)
            self.start_position = parent.start_position
            self.portals = copy.deepcopy(parent.portals)
            self.orange_boxes = copy.deepcopy(parent.orange_boxes)
        else:
            self.picked_boxes = []
            self.portals = []
            self.orange_boxes = []
            self.start_position = self.board.find_position(self.get_agent_code())
        if self.position in board.boxes and self.position not in self.picked_boxes:
            self.picked_boxes.append(self.position)
        if self.position in board.orange_boxes and self.position not in self.orange_boxes and len(self.picked_boxes) > len(self.orange_boxes):
            self.orange_boxes.append(self.position)

        self.has_boxes = (len(self.picked_boxes) == len(board.boxes) and len(self.orange_boxes) == len(board.orange_boxes)) or (parent is not None and parent.has_boxes)
        #self.has_boxes = (len(self.picked_boxes) == 3 and len(self.orange_boxes) == 2) or (parent is not None and parent.has_boxes) 3. zadatak b)
        #self.has_boxes = (len(self.picked_boxes) == 3) or (parent is not None and parent.has_boxes) # 2.zadatak a)

    def get_agent_code(self):
        return 'r'

    def get_agent_goal_code(self):
        return 'g'

    def get_portal_code(self):
        return 'p'

    def get_fire_code(self):
        return 'f'

    def get_current_cost(self):
        return self.depth

    def get_cost(self):
        return ((self.position[0] - self.goal_position[0])**2 + (self.position[1] - self.goal_position[1])**2)**0.5

    def get_fire_cost(self):
        return 7*((self.position[0] - self.fire_position[0])**2 + (self.position[1] - self.fire_position[1])**2)**0.5 # 3 zadatak b)

    def is_state_portal(self):
        return  self.position == self.board.find_position(self.get_portal_code())

    def get_legal_positions(self, picked_box, initial_state):
        # d_rows (delta rows), d_cols (delta columns)
        # moguci smerovi kretanja robota (desno, levo, dole, gore)
        """
        if picked_box: # 1 zadatak a)
            d_rows = [0, 0, 1, -1, 1, -1, 1, -1]
            d_cols = [1, -1, 0, 0, 1, 1, -1, -1]
        else:
            d_rows = [0, 0, 1, -1]
            d_cols = [1, -1, 0, 0]
        """
        d_rows = [0, 0, 1, -1]
        d_cols = [1, -1, 0, 0]
        drows = []
        dcols = []


        row, col = self.position  # trenutno pozicija
        for r in range(self.board.rows):
            drows.extend([0, 0, r, -r, r, -r, r, -r])
            dcols.extend([r, -r, 0, 0, r, r, -r, -r])

        #print(row, col)
        new_positions = []
        #print(initial_state)
        for d_row, d_col in zip(drows, dcols):  # za sve moguce smerove
            new_row = row + d_row  # nova pozicija po redu
            new_col = col + d_col  # nova pozicija po koloni
            # ako nova pozicija nije van table i ako nije zid ('w'), ubaci u listu legalnih pozicija
            if 0 <= new_row < self.board.rows and 0 <= new_col < self.board.cols and self.board.data[new_row][new_col] != 'w':
                new_positions.append((new_row, new_col))

        portal_positions = self.board.find_elements(self.get_portal_code())
        if self.position in portal_positions: # 2 zadatak b)
            r = random.random()
            if r < 0.7:
                portal_positions.remove(self.position)
                new_positions.extend(portal_positions)
            else:
                new_positions.clear()
                self.portals.append(self.position)
                new_positions.append(self.start_position)
        return new_positions

    def is_final_state(self):
        return self.position == self.goal_position and self.has_boxes

    def unique_hash(self):
        return str(self.position) + str(self.picked_boxes) + str(self.portals) + str(self.orange_boxes)
