import pyserial

def posx( xc ):
    q= f"G91 X+{xc} Y+{xc}"
    #send q in serial

def negx( xc ):
    q= f"G91 X-{xc} Y-{xc}"
    #send q in serial

def posy( yc ):
    q= f"G91 X-{yc} Y+{yc}"
    #send q in serial

def negy( yc ):
    q= f"G91 X+{yc} Y-{yc}"
    #send q in serial
