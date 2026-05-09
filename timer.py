import time
from datetime import datetime

class TaskTimer:
    def __init__(self):
        self.start_time = time.time()
        self.last_checkpoint = self.start_time
        self.records = [] 

    def reset(self):
        """Reset the timer and start a new round of tasks"""
        self.start_time = time.time()
        self.last_checkpoint = self.start_time
        self.records = []
        print("\n=== Task Timer Started ===")

    def lap(self, step_name):
        """Checkpoint the time from the last step to now and print it"""
        now = time.time()
        duration = now - self.last_checkpoint
        self.records.append((step_name, duration))
        self.last_checkpoint = now
        print(f"[Task Timer] {step_name:<15} Duration: {duration:.2f}s")

    def save_to_txt(self, filename="agv_log.txt", note=""):
        """
        Save this round of data to a txt file
        filename: file name
        note: optional note (for example '3rd test' or 'after changing the battery')
        """
        total_time = sum(t for _, t in self.records)
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        try:
            with open(filename, "a", encoding="utf-8") as f:
                f.write(f"\n{'='*10} Record Time: {timestamp} {note} {'='*10}\n")

                for name, duration in self.records:
                    f.write(f"{name:<20} : {duration:.2f}s\n")
                
                f.write("-" * 35 + "\n")
                f.write(f"{'Total Time':<20} : {total_time:.2f}s\n")
                f.write("=" * 46 + "\n")
                
            print(f"Data saved successfully to {filename}")
        
        except Exception as e:
            print(f"Failed to save data to file: {e}")
