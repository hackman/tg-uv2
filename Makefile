all:
	gcc -o qs-tg-uv2 qs-tg-uv2.c

clean:
	rm -f qs-tg-uv2

test:
	./qs-tg-uv2 -h
