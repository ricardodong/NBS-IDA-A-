def isNotNum(value):
    try:
        value + 1
    except TypeError:
        return True
    else:
        return False

def inHashTable(hashtable, key):
    try:
        a = hashtable[key]
    except KeyError:
        return False, 0
    else:
        return True, a

def swap(a,b):
    return b,a