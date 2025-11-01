# pins.py Dicts mapping GP and Pico pin nos onto monitor ID's

# trig = monitor.trigger(gp(10))
# trig causes output on Pico GP10
def gp(g):
    if 3 <= g <= 22:
        return g -3
    elif 26  <= g <= 27:
        return g -6
    raise ValueError("Invalid GPIO no. for monitor")
# trig = monitor.trigger(pico(6)
# Output on Pico PCB pin 6 (GP4)
def pico(p):
    pcb = {5:0, 6:1, 7:2, 9:3, 10:4, 11:5, 12:6, 14:7, 15:8, 16:9, 17:10, 19:11}
    pcb.update({20:12, 21:13, 22:14, 24:15, 25:16, 26:17, 27:18, 29:19, 31:20, 32:21})
    try:
        return pcb[p]
    except KeyError:
        raise ValueError("Invalid Pico PCB pin no. for monitor")
