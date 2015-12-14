#! /usr/bin/python

"""Module docstring.

This serves as a long usage message.
"""
import sys
import getopt

def main():
    # parse command line options
    try:
        opts, args = getopt.getopt(sys.argv[1:], "h", ["help"])
    except getopt.error, msg:
        print msg
        print "for help use --help"
        return 2
    # process options
    for o, a in opts:
        if o in ("-h", "--help"):
            print __doc__
            return 0
    # process arguments
    for arg in args:
        process(arg) # process() is defined elsewhere

if __name__ == "__main__":
    sys.exit(main())
