doc: README.md

README.md : README.sec
	sectxt.py -k $^ > $@
