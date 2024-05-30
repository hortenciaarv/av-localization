from class_bug0 import Bug0

class Bug2(Bug0):
  
    def calculate_line_param(self):
        self.m_line = (self.goal_y - self.pos_y)/ (self.goal_x - self.pos_x)
        self.b_line = self.goal_y - self.m_line * self.goal_x

    def calculate_line(self):
        return self.m_line * self.pos_x + self.b_line






