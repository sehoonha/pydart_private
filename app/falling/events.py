class Event(object):
    def __init__(self, _name, _after):
        self.name = _name
        self.after = _after

    def __repr__(self):
        return "<Event %s after %d>" % (self.name, self.after)


class Handler(object):
    def __init__(self):
        self.events = []

    def step(self):
        for e in self.events:
            e.after -= 1

    def clear(self):
        self.events = []
        
    def push(self, _name, _after = 0):
        if len([e for e in self.events if e.name == _name]) > 0:
            return
        self.events += [Event(_name, _after)]

    def pop(self):
        activated = []
        not_activated = []
        for e in self.events:
            if e.after <= 0:
                activated += [e]
            else:
                not_activated += [e]
        self.events = not_activated
        return activated

    def size(self):
        return len(self.events)
    
    def __repr__(self):
        return "{" + ",".join([str(e) for e in self.events]) + "}"
    
