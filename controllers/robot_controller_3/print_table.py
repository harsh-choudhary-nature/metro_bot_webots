import pickle

# class pqr:
#     def __init__(self):
#         self.a = 5

# def abc(l,o):
#     l[1] = 5
#     o.a = 6

# l = [1,2]
# o = pqr()
# o.a = 55
# abc(l,o)
# print(l,o.a)

table = 0
with open("q_table.pkl","rb") as fh:
    table = pickle.load(fh)

with open("table_file.txt","w") as fh:
    for state in table:
        fh.write(f"{state},stop: {table[state]['stop']},move: {table[state]['move']}\n")
        # print(state,"\tstop: ",table[state]["stop"],"\tmove: ",table[state]["move"])