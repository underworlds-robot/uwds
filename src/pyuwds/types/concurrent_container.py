from threading import Lock

class ConcurrentContainer(object):

    def __init__(self):
        self.__map = {}
        self.__mutex = Lock()

    def _lock(self):
        self.__mutex.acquire()

    def _unlock(self):
        self.__mutex.release()

    def update(self, ids, elements):
        self._lock()
        for el in zip(ids, elements):
            self.__map[el[0]] = el[1]
        self._unlock()

    def remove(self, ids):
        self._lock()
        for id in ids:
            if self.has(id):
                del self.__map[id]
        self._unlock()

    def is_empty(self):
        return self.get_size() > 0
    
    def get_size(self):
        return len(self.__map)

    def reset(self):
        self._lock()
        self.__map.clear()
        self._unlock()

    def has(self, id):
        return id in self.ids()

    def __getitem__(self, key):
        return self.__map[key]

    def __contains__(self, key):
        return self.has(key)

    def ids(self):
        return self.__map.keys()
    