from strategy_test_g2s import G2S

class Parent:
    def __init__(self):
        pass
    def init_on_enter(self):
        pass
    def init_on_leave(self):
        pass
    def init_to_end_ontr(self):
        pass
    def init_to_end_guard(self):
        pass

parent = Parent()
g2s = G2S(parent)
g2s.debug = True
g2s.start()