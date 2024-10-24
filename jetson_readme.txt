If you receive this kind of error:

ImportError: /lib/aarch64-linux-gnu/libgomp.so.1: cannot allocate memory in static TLS block

Try to run this comands:


$ source ~/.bashrc

$ export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1
