import threading, time

class Camera:
    def __init__(self, name, width, height, pix_Per_M):
        self.name = name
        self.width = width
        self.height = height
        self.pix_Per_M = pix_Per_M
        
        #Threading 
        self._ready = False
        self.thread_ready = threading.Event()
        self.thread_ready.clear()

        #Sync camera frames
        self._lock = threading.Lock()
        self._latest  = None 
        pass
    
    def get_latest(self):
        """Return the most recent: RGBD {'color','depth','ts'} 
        Ther {'image', 'ts'} or None if nothing yet."""
        self._lock.acquire()
        item = self._latest
        self._lock.release()
        return item
    
    def is_ready(self):
        return self._ready

    def stop_stream(self):
        self.thread_ready.clear()
        
    def start_stream(self):
        self.thread_ready.set()
        while not self.get_latest():
            print(f"Waiting for {self.name} response...")
            time.sleep(0.5)
    
    