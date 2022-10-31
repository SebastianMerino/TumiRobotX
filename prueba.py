import time
import threading

LINE_UP = '\033[1A'
LINE_CLEAR = '\x1b[2K'

i = 0
stop_PWM = False
def show_PWM():
	while not stop_PWM:
		global i
		print(i)
		time.sleep(0.2)
		print(LINE_UP, end=LINE_CLEAR)
		i += 1

print('\n')
t = threading.Thread(target=show_PWM, daemon=True)
t.start()

time.sleep(10)
print('gaaa\n')
stop_PWM = True