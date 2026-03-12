import time
from datetime import datetime

class TaskTimer:
    def __init__(self):
        self.start_time = time.time()
        self.last_checkpoint = self.start_time
        self.records = [] # Store a list of steps (step name, time taken).

    def reset(self):
        """Reset the timer and start a new round of tasks."""
        self.start_time = time.time()
        self.last_checkpoint = self.start_time
        self.records = []
        print("\n=== Task timer starts ===")

    def lap(self, step_name):
        """Tracking: Record the time taken from the previous step to the present."""
        now = time.time()
        duration = now - self.last_checkpoint
        self.records.append((step_name, duration))
        self.last_checkpoint = now
        # Print it out in real time so you can easily see the screen.
        print(f"[Timing] {step_name:<15} time-consuming: {duration:.2f}s")

    def save_to_txt(self, filename="agv_log.txt", note=""):
        """
       Save the data from this round to a txt file.
        filename: filename
        note: Optional remarks (e.g., '3rd test' or 'After battery replacement')
        """
        total_time = sum(t for _, t in self.records)
        # Get the current date and time, for example, "2023-10-27 15:30:00".
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        try:
            # mode='a' This indicates append mode, which will not overwrite existing content.
            with open(filename, "a", encoding="utf-8") as f:
                f.write(f"\n{'='*10} Recording time: {timestamp} {note} {'='*10}\n")
                
                # Write down the details of each step.
                for name, duration in self.records:
                    # <20 Write down the details of each step.
                    f.write(f"{name:<20} : {duration:.2f}s\n")
                
                f.write("-" * 35 + "\n")
                f.write(f"{'Total time':<20} : {total_time:.2f}s\n")
                f.write("=" * 46 + "\n")
                
            print(f"The data has been successfully saved. {filename}")
            
        except Exception as e:
            print(f"File saving failed: {e}")