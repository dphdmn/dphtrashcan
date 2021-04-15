import discord
from discord.utils import get
from re import match
from discord.ext import commands, tasks
import os
from asyncio import sleep
import subprocess
import urllib.request
import html2text
import textwrap
import socket
import random
import time
import subprocess
import traceback
import time
import asyncio
import os
import platform
from itertools import count
from multiprocessing import Process
from array import *
import datetime
import math
from decimal import *

def mod_date(path_to_file):
   stat = os.stat(path_to_file)
   return datetime.datetime.fromtimestamp(stat.st_mtime)

client = discord.Client()

def sumAm(a,b, cumA):
    val=0
    if a>0:
        val = cumA[b]-cumA[a-1]
    else:
        val = cumA[b]
    return val


def getProbText(f1, f2, pzlName, nscr):
    f1 = int(f1)
    f2 = int(f2)
    nscr = int(nscr)
    if nscr == 0:
        nscr = 1
    num44 = [1, 2, 4, 10, 24, 54, 107, 212, 446, 946, 1948, 3938, 7808, 15544, 30821, 60842, 119000, 231844, 447342,
             859744, 1637383, 3098270, 5802411, 10783780, 19826318, 36142146, 65135623, 116238056, 204900019, 357071928,
             613926161, 1042022040, 1742855397, 2873077198, 4660800459, 7439530828, 11668443776, 17976412262,
             27171347953, 40271406380, 58469060820, 83099401368, 115516106664, 156935291234, 208207973510, 269527755972,
             340163141928, 418170132006, 500252508256, 581813416256, 657076739307, 719872287190, 763865196269,
             784195801886, 777302007562, 742946121222, 683025093505, 603043436904, 509897148964, 412039723036,
             317373604363, 232306415924, 161303043901, 105730020222, 65450375310, 37942606582, 20696691144, 10460286822,
             4961671731, 2144789574, 868923831, 311901840, 104859366, 29592634, 7766947, 1508596, 272198, 26638, 3406,
             70, 17]
    num33 = [1, 2, 4, 8, 16, 20, 39, 62, 116, 152, 286, 396, 748, 1024, 1893, 2512, 4485, 5638, 9529, 10878, 16993,
             17110, 23952, 20224, 24047, 15578, 14560, 6274, 3910, 760, 221, 2]
    maxStates4 = 10461394944000
    maxStates3 = 181440
    if pzlName == "3x3":
        numTable = num33
        pzlMaxStates = maxStates3
    if pzlName == "4x4":
        numTable = num44
        pzlMaxStates = maxStates4
    limitNum = len(numTable)-1
    #print(limitNum)
    if f1 < 0:
        f1 = 0
    if f2 < 0:
        f1 = 0
        f2 = 0
    if f1 > limitNum:
        f1 = limitNum
        f2 = limitNum
    if f2 > limitNum:
        f2 = limitNum
    if f1 > f2:
        f1, f2 = f2, f1
    out = ""
    cumP = []
    cumA = []
    sum = 0
    maxV = len(numTable)
    for i in range(0, maxV):
        sum += numTable[i]
        cumA.append(sum)
        cumP.append(100 * (sum / pzlMaxStates))
    rp = getRangeP(f1, f2, cumP)
    out += "The probability of " + bl(pzlName) + " puzzle being optimally solved in "
    if (f1 != f2):
        out += "range from " + bl(f1) + " to " + bl(f2) + " moves is "
    else:
        out += "*exactly* " + bl(f1) + " moves is "
    out += editP(rp)

    out += "\n(since there are " + bl(sumAm(f1, f2, cumA)) + " states in that range of total " + str(pzlMaxStates) + " states of this puzzle)"
    pSc = (1 - ((1 - rp / 100) ** nscr)) * 100
    #sc = str(pSc)
    out += "\nThe probability of getting at least one scramble within that range after " + bl(str(nscr)) + " scrambles is " + editP(pSc)
    #print(nscr)
    #if nscr > 100:
    maxi=1
    maxFind = min(1000, nscr)
    for i in range (1, maxFind):
        ber = bernully(i,nscr,rp)
        maxi=i
        if ber != "[Very small]":
            break
    if bernully(maxi,nscr,rp) == "[Very small]":
        out += "```Sorry, can't find chance of getting scramble exactly " + str(maxi) + " times after " + str(nscr) + " solves, it's still very small```"
    else:
        out += "```"
        lasti=maxi+17
        for i in range(maxi, lasti):
            berv=bernully(i, nscr, rp)
            if (berv == "1 in i"):
                berv="[Very big]"
            out += "\nChance of getting scramble exactly " + str(i) + " times after " + str(nscr) + " solves is " + berv
        out += "```"
    out += "\n\nCheck this: https://dphdmn.github.io/15puzzleprob/"
    return out

def bernully(k, n, p): #k times in n tests, prob = p
    p = p/100
    x1 = math.comb(n, k)
    x2 = (p ** k)
    x3 = ((1-p) ** (n-k))
    value = Decimal(x1)*Decimal(x2)*Decimal(x3)
   # print(bl(str(round(float(Decimal(100) / Decimal(value)), 0))))
    return editP(value*100).replace("*","")

def getRangeP(a, b, cumP):
    val = 0
    if a > 0:
        val = cumP[b] - cumP[a - 1]
    else:
        val = cumP[b]
    return val


def bl(s):
    return "**" + str(s) + "**"


def editP(rp):
    out = ""
    if rp == 0:
        return "[Very small]"
    if rp < 1:
        out += "1 in " + bl(str(round(float(Decimal(100) / Decimal(rp)), 0))[:-2])
    else:
        out += bl(str(round(float(rp), 2))) + "%"
    return out


class IDAStar:
    def __init__(self, h, neighbours):
        """ Iterative-deepening A* search.
 
        h(n) is the heuristic that gives the cost between node n and the goal node. It must be admissable, meaning that h(n) MUST NEVER OVERSTIMATE the true cost. Underestimating is fine.
 
        neighbours(n) is an iterable giving a pair (cost, node, descr) for each node neighbouring n
        IN ASCENDING ORDER OF COST. descr is not used in the computation but can be used to
        efficiently store information about the path edges (e.g. up/left/right/down for grids).
        """
 
        self.h = h
        self.neighbours = neighbours
        self.FOUND = object()
        self.tic = time.perf_counter()
 
    def solve(self, root, is_goal, max_cost=None):
        """ Returns the shortest path between the root and a given goal, as well as the total cost.
        If the cost exceeds a given max_cost, the function returns None. If you do not give a
        maximum cost the solver will never return for unsolvable instances."""
 
        self.is_goal = is_goal
        self.path = [root]
        self.is_in_path = {root}
        self.path_descrs = []
        self.nodes_evaluated = 0
 
        bound = self.h(root)
        
        while True:  
            t = self._search(0, bound)
            if t is self.FOUND: return self.path, self.path_descrs, bound, self.nodes_evaluated
            if t is None: return None
            #tok = time.perf_counter()
            #print(str(tok-tic)+ "____"+ str(t))
            #if (tok-tic) > 60: return "timeout","timeout","timeout","timeout"
            bound = t
 
    def _search(self, g, bound):
        self.nodes_evaluated += 1
 
        node = self.path[-1]
        f = g + self.h(node)
        if f > bound: return f
        if self.is_goal(node): return self.FOUND
 
        m = None # Lower bound on cost.
        for cost, n, descr in self.neighbours(node):
            if n in self.is_in_path: continue
            tok = time.perf_counter()
            dif = tok-self.tic
            #print(str(dif))
            if dif > 60:
                dif2 = (f"Elapsed time: {dif:0.2f} seconds")
                raise Exception("TimeOut. Probably your scramble is too hard for me :(\nScramble is at least "+str(bound) + " moves.\n"+dif2)
            self.path.append(n)
            self.is_in_path.add(n)
            self.path_descrs.append(descr)
            t = self._search(g + cost, bound)
 
            if t == self.FOUND: return self.FOUND
            if m is None or (t is not None and t < m): m = t
 
            self.path.pop()
            self.path_descrs.pop()
            self.is_in_path.remove(n)
 
        return m
 
def slide_solved_state(n):
    return tuple(i % (n*n) for i in range(1, n*n+1))
 
def slide_randomize(p, neighbours):
    for _ in range(len(p) ** 2):
        _, p, _ = random.choice(list(neighbours(p)))
    return p
 
def slide_neighbours(n):
    movelist = []
    for gap in range(n*n):
        x, y = gap % n, gap // n
        moves = []
        if x > 0: moves.append(-1)    # Move the gap left.
        if x < n-1: moves.append(+1)  # Move the gap right.
        if y > 0: moves.append(-n)    # Move the gap up.
        if y < n-1: moves.append(+n)  # Move the gap down.
        movelist.append(moves)
 
    def neighbours(p):
        gap = p.index(0)
        l = list(p)
 
        for m in movelist[gap]:
            l[gap] = l[gap + m]
            l[gap + m] = 0
            yield (1, tuple(l), (l[gap], m))
            l[gap + m] = l[gap]
            l[gap] = 0
 
    return neighbours
 
def slide_print(p):
    n = int(round(len(p) ** 0.5))
    l = len(str(n*n))
    for i in range(0, len(p), n):
        print(" ".join("{:>{}}".format(x, l) for x in p[i:i+n]))
 
def encode_cfg(cfg, n):
    r = 0
    b = n.bit_length()
    for i in range(len(cfg)):
        r |= cfg[i] << (b*i)
    return r
 
 
def gen_wd_table(n):
    goal = [[0] * i + [n] + [0] * (n - 1 - i) for i in range(n)]
    goal[-1][-1] = n - 1
    goal = tuple(sum(goal, []))
 
    table = {}
    to_visit = [(goal, 0, n-1)]
    while to_visit:
        cfg, cost, e = to_visit.pop(0)
        enccfg = encode_cfg(cfg, n)
        if enccfg in table: continue
        table[enccfg] = cost
 
        for d in [-1, 1]:
            if 0 <= e + d < n:
                for c in range(n):
                    if cfg[n*(e+d) + c] > 0:
                        ncfg = list(cfg)
                        ncfg[n*(e+d) + c] -= 1
                        ncfg[n*e + c] += 1
                        to_visit.append((tuple(ncfg), cost + 1, e+d))
 
    return table
 
def slide_wd(n, goal):
    wd = gen_wd_table(n)
    goals = {i : goal.index(i) for i in goal}
    b = n.bit_length()
 
    def h(p):
        ht = 0 # Walking distance between rows.
        vt = 0 # Walking distance between columns.
        d = 0
        for i, c in enumerate(p):
            if c == 0: continue
            g = goals[c]
            xi, yi = i % n, i // n
            xg, yg = g % n, g // n
            ht += 1 << (b*(n*yi+yg))
            vt += 1 << (b*(n*xi+xg))
 
            if yg == yi:
                for k in range(i + 1, i - i%n + n): # Until end of row.
                    if p[k] and goals[p[k]] // n == yi and goals[p[k]] < g:
                        d += 2
 
            if xg == xi:
                for k in range(i + n, n * n, n): # Until end of column.
                    if p[k] and goals[p[k]] % n == xi and goals[p[k]] < g:
                        d += 2
 
        d += wd[ht] + wd[vt]
 
        return d
    return h
    
@client.event
async def on_ready():
    print('We have logged in as {0.user}'.format(client))
    
async def superSolve(arr, thingy):
    print("doing things") 
    tic = time.perf_counter()
    solved_state = slide_solved_state(4)
    neighbours = slide_neighbours(4)
    is_goal = lambda p: p == solved_state
    slide_solver = IDAStar(slide_wd(4, solved_state), neighbours)
    path, moves, cost, num_eval = slide_solver.solve(arr, is_goal, 80)
    #slide_print(arr)
    toc = time.perf_counter()
    dif = (f"Solved in {toc - tic:0.2f} seconds")
    sol="".join({-1: "R", 1: "L", -4: "D", 4: "U"}[move[1]] for move in moves)
    return ("Solution for "+thingy+ "\n"+sol+"\n"+str(cost)+" moves\n"+dif)
    #print(cost)     
@client.event
async def on_message(message):
    namea = str(message.author)
    if message.author == client.user:
        return
    if '!testy' in message.content.lower():   
        try:
            thingy = message.content[7:]
            arr = tuple([int(a) for x in thingy.split("/") for a in x.split()])   
            try:
                ans = await asyncio.wait_for(superSolve(arr, thingy), timeout=0.000001)
                await message.channel.send(ans)
            except asyncio.TimeoutError:
                print('timeout!')
        except:
            await message.channel.send("Something is wrong\n```"+traceback.format_exc()+"```")    
    if 'egg' in message.content.lower():
        await message.channel.send('Egg!')
        await message.add_reaction('\N{EGG}')
        await message.add_reaction("eg:800488248967168040")
        await message.add_reaction("eggg:800490913263517706")
        await message.add_reaction("eggon:807541711847817229")
    if 'яйцо' in message.content.lower():
        await message.channel.send('Яйцо!')
        await message.add_reaction('\N{EGG}')
        await message.add_reaction("eg:800488248967168040")
        await message.add_reaction("eggg:800490913263517706")
        await message.add_reaction("eggon:807541711847817229")    
    if 'pls' in message.content.lower():
        await message.add_reaction('eff:803888415858098217')
   # if 'vovker' in namea:
   #     await message.add_reaction('vodker:807538254957510677')
   # if 'eff' in namea:
   #     await message.add_reaction('eff:803888415858098217')
    if match("<@!?809437517564477522>", message.content) is not None:
        await message.channel.send("You are egg, "+message.author.mention)
    if 'fuck you' in message.content.lower():
        await message.channel.send("no u, "+message.author.mention)
    if '!spam' in message.content.lower():
        if message.author.guild_permissions.administrator:
            shit = message.content[6:]
            msg = ""
            for x in range(3000):
                msg += shit+" "
            spam.start(message.channel, msg[:2000])
    if '!stop' in message.content.lower():
        if message.author.guild_permissions.administrator:
            spam.cancel()
    if '!getwr' in message.content:
        try:
            fp = urllib.request.urlopen("http://slidysim.000webhostapp.com/leaderboard/records_all.html")
            mybytes = fp.read()
            mystr = mybytes.decode("utf8")
            mystr = html2text.html2text(mystr)
            mystr = mystr.splitlines()
            fp.close()
            wrsize = message.content[7:]+" "
            matching = [s for s in mystr if wrsize in s]
            if len(matching) == 0:
                await message.channel.send("Sorry, i can't find anything :(\nTry this: http://bit.ly/wrspage")
            else:
                out = matching[0]
                await message.channel.send(out)
        except:
            await message.channel.send("Something is wrong\n```"+traceback.format_exc()+"```")  
    if '!wrsby' in message.content:
        try:
            fp = urllib.request.urlopen("http://slidysim.000webhostapp.com/leaderboard/records_all.html")
            mybytes = fp.read()
            mystr = mybytes.decode("utf8")
            mystr = html2text.html2text(mystr)
            mystr = mystr.splitlines()
            fp.close()
            username = message.content[7:]
            matching = [s for s in mystr if username in s]
            my_string = ';\n'.join(matching)
            if len(matching) == 0:
                await message.channel.send("Sorry, i can't find anything :(\nTry this: http://bit.ly/wrspage")
            else:
                if len(my_string)>1700 and not message.author.guild_permissions.kick_members:
                    await message.channel.send("Sorry, egg, this message is too long :(\nTry this: http://bit.ly/wrspage")
                else:
                    news = ""
                    for i in matching:
                        news+=i+"\n"
                        if(len(news)>1700):
                            await message.channel.send("```"+news+"```")
                            news=""
                    await message.channel.send("```"+news+"```")        
        except:
            await message.channel.send("Something is wrong\n```"+traceback.format_exc()+"```")
    if '!getpb' in message.content:
            with open('shit.txt', 'r') as file:
                mystr = file.read().lower()#.replace('\n', '')
            #print(mystr)
            filedate = str(mod_date("shit.txt")).split(' ')[0]
            mystr = mystr.splitlines()
            contentArray = message.content.lower().split(' ')
            #print(contentArray)
            username = contentArray[1]
            matching = [s for s in mystr if username in s]
            my_string = matching[0]
            my_string = my_string.split('\t')

            #print(my_string)
            bad = False
            try:
                for i in range(1, len(my_string)):
                    try:
                        number = float(my_string[i])
                        intpart = int(math.floor(number))
                        decimals = round(number - intpart,3)
                        x = str(datetime.timedelta(seconds=intpart)) + str(decimals)[1:]
                    except:
                        x = ""
                    if x != "":
                        if int(intpart) > 60:
                            my_string[i] = my_string[i] + " (" + x[2:] + ")"
                puzzle = contentArray[2]
                outputString = "PBs for user **" + my_string[0] + "** at the puzzle "
                if puzzle == "3" or puzzle == "3x3":
                    outputString += "3x3\n```"
                    outputString += "ao5: " + my_string[1] + "\n"
                    outputString += "ao12: " + my_string[2] + "\n"
                    outputString += "ao50: " + my_string[3] + "\n"
                    outputString += "ao100: " + my_string[4] + "\n"
                    outputString += "x10 marathon: " + my_string[5] + "\n"
                    outputString += "x42 marathon: " + my_string[6] + "\n```"
                elif puzzle == "4" or puzzle == "4x4":
                    outputString += "4x4\n```"
                    outputString += "single: " + my_string[7] + "\n"
                    outputString += "ao5: " + my_string[8] + "\n"
                    outputString += "ao12: " + my_string[9] + "\n"
                    outputString += "ao50: " + my_string[10] + "\n"
                    outputString += "ao100: " + my_string[11] + "\n"
                    outputString += "x10 marathon: " + my_string[12] + "\n"
                    outputString += "x42 marathon: " + my_string[13] + "\n"
                    outputString += "4x4 - 2x2 relay: " + my_string[14] + "\n```"
                elif puzzle == "5" or puzzle == "5x5":
                    outputString += "5x5\n```"
                    outputString += "single: " + my_string[15] + "\n"
                    outputString += "ao5: " + my_string[16] + "\n"
                    outputString += "ao12: " + my_string[17] + "\n"
                    outputString += "ao50: " + my_string[18] + "\n"
                    outputString += "5x5 - 2x2 relay: " + my_string[19] + "\n```"
                elif puzzle == "6" or puzzle == "6x6":
                    outputString += "6x6\n```"
                    outputString += "single: " + my_string[20] + "\n"
                    outputString += "ao5: " + my_string[21] + "\n"
                    outputString += "ao12: " + my_string[22] + "\n"
                    outputString += "6x6 - 2x2 relay: " + my_string[23] + "\n```"
                elif puzzle == "7" or puzzle == "7x7":
                    outputString += "7x7\n```"
                    outputString += "single: " + my_string[24] + "\n"
                    outputString += "ao5: " + my_string[25] + "\n"
                    outputString += "7x7 - 2x2 relay: " + my_string[26] + "\n```"
                elif puzzle == "8" or puzzle == "8x8":
                    outputString += "8x8\n```"
                    outputString += "single: " + my_string[27] + "\n"
                    outputString += "ao5: " + my_string[28] + "\n```"
                elif puzzle == "9" or puzzle == "9x9":
                    outputString += "9x9\n```"
                    outputString += "single: " + my_string[29] + "\n```"
                elif puzzle == "10" or puzzle == "10x10":
                    outputString += "10x10\n```"
                    outputString += "single: " + my_string[30] + "\n```"
                else:
                    await message.channel.send("Can't find this puzzle, make sure it's from 3x3 to 10x10.\nFor other pbs check leaderboard in slidysim.")
                    bad=True
                if not bad:
                    outputString += "\nLast time update: " + filedate
                    #print(outputString)
                    await message.channel.send(outputString)
            except:
                await message.channel.send("Please specify the puzzle size, for example: !getpb dphdmn 4x4")
    if '!getreq' in message.content:
        with open('tiers.txt', 'r') as file:
            mystr = file.read().lower()  # .replace('\n', '')
        # print(mystr)
        mystr = mystr.splitlines()
        contentArray = message.content.lower().split(' ')
        # print(contentArray)
        username = contentArray[1]
        matching = [s for s in mystr if username in s]
        my_string = matching[0]
        my_string = my_string.split('\t')
        # print(my_string)

        bad = False
        try:
            puzzle = contentArray[2]
            outputString = "Requirement for tier **" + my_string[0] + "** at the puzzle "
            for i in range(1, len(my_string)):
                try:
                    number = float(my_string[i])
                    intpart = int(math.floor(number))
                    decimals = round(number - intpart, 3)
                    x = str(datetime.timedelta(seconds=intpart)) + str(decimals)[1:]
                except:
                    x = ""
                if x != "":
                    if int(intpart) > 60:
                        my_string[i] = my_string[i] + " (" + x[2:] + ")"

            if puzzle == "3" or puzzle == "3x3":
                outputString += "3x3\n```"
                outputString += "ao5: " + my_string[1] + "\n"
                outputString += "ao12: " + my_string[2] + "\n"
                outputString += "ao50: " + my_string[3] + "\n"
                outputString += "ao100: " + my_string[4] + "\n"
                outputString += "x10 marathon: " + my_string[5] + "\n"
                outputString += "x42 marathon: " + my_string[6] + "\n```"
            elif puzzle == "4" or puzzle == "4x4":
                outputString += "4x4\n```"
                outputString += "single: " + my_string[7] + "\n"
                outputString += "ao5: " + my_string[8] + "\n"
                outputString += "ao12: " + my_string[9] + "\n"
                outputString += "ao50: " + my_string[10] + "\n"
                outputString += "ao100: " + my_string[11] + "\n"
                outputString += "x10 marathon: " + my_string[12] + "\n"
                outputString += "x42 marathon: " + my_string[13] + "\n"
                outputString += "4x4 - 2x2 relay: " + my_string[14] + "\n```"
            elif puzzle == "5" or puzzle == "5x5":
                outputString += "5x5\n```"
                outputString += "single: " + my_string[15] + "\n"
                outputString += "ao5: " + my_string[16] + "\n"
                outputString += "ao12: " + my_string[17] + "\n"
                outputString += "ao50: " + my_string[18] + "\n"
                outputString += "5x5 - 2x2 relay: " + my_string[19] + "\n```"
            elif puzzle == "6" or puzzle == "6x6":
                outputString += "6x6\n```"
                outputString += "single: " + my_string[20] + "\n"
                outputString += "ao5: " + my_string[21] + "\n"
                outputString += "ao12: " + my_string[22] + "\n"
                outputString += "6x6 - 2x2 relay: " + my_string[23] + "\n```"
            elif puzzle == "7" or puzzle == "7x7":
                outputString += "7x7\n```"
                outputString += "single: " + my_string[24] + "\n"
                outputString += "ao5: " + my_string[25] + "\n"
                outputString += "7x7 - 2x2 relay: " + my_string[26] + "\n```"
            elif puzzle == "8" or puzzle == "8x8":
                outputString += "8x8\n```"
                outputString += "single: " + my_string[27] + "\n"
                outputString += "ao5: " + my_string[28] + "\n```"
            elif puzzle == "9" or puzzle == "9x9":
                outputString += "9x9\n```"
                outputString += "single: " + my_string[29] + "\n```"
            elif puzzle == "10" or puzzle == "10x10":
                outputString += "10x10\n```"
                outputString += "single: " + my_string[30] + "\n```"
            else:
                await message.channel.send("Can't find this puzzle, make sure it's from 3x3 to 10x10.")
                bad = True
            if not bad:
                await message.channel.send(outputString)
        except:
            await message.channel.send("Please specify the puzzle size, for example: !getreq ascended 4x4")
    if '!getprob' in message.content.lower(): #!getprob 4x4 30 40 1000, or !getprob 4x4 30 40, or !getprob 4x4 30
        try:
            examples="- !getprob <puzzle> <moves> [<moves>, <amount> || <amount] - get probability of getting N moves optimal scramble\nCommand examples:\n```!getprob 4x4 30 - get probability for 4x4 in 30 moves\n!getprob 4x4 30 40 - get probability for 4x4 from 30 to 40 moves\n!getprob 4x4 30 40 _1000 - from 30 to 40 moves, repeat 1000 times (default 100)\n!getprob 4x4 30 _1000 - 30 moves, repeat 1000 times\n!getprob 3x3 20 - 3x3 puzzle```\n"
            contentArray = message.content.lower().split(' ')
            arraylen = len(contentArray)
            if arraylen == 1:
                await message.channel.send(examples)
            elif arraylen == 2:
                examples = "Command should have at least 3 words in it.\n" + examples
                await message.channel.send(examples)
            elif arraylen == 3:
                pzlName = contentArray[1]
                if pzlName == "3x3" or pzlName == "4x4":
                    num = contentArray[2]
                    if num.isdigit():
                        f1 = num
                        f2 = num
                        await message.channel.send(getProbText(f1, f2, pzlName, 100))
                    else:
                        examples = "Something is wrong with your range number, most be positive integer.\n" + examples
                        await message.channel.send(examples)
                else:
                    examples = "Something is wrong with your puzzle size (must be 4x4 or 3x3)\n" + examples
                    await message.channel.send(examples)
            else:
                pzlName = contentArray[1]
                if pzlName == "3x3" or pzlName == "4x4":
                    if arraylen == 4: #expect number and amount rep OR number and second number
                        num = contentArray[2]
                        if num.isdigit():
                            otherThing=contentArray[3]
                            if otherThing[:1] == "_": #thiking that this is number and amount of rep
                                num2 = otherThing[1:]
                                if num2.isdigit():
                                    await message.channel.send(getProbText(num, num, pzlName, num2))
                                else:
                                    examples = "Something is wrong with your range number, most be positive integer.\n" + examples
                                    await message.channel.send(examples)
                            else:#thinking that this number is just the 2nd range number
                                num2 = otherThing
                                if num2.isdigit():
                                    await message.channel.send(getProbText(num, num2, pzlName, 100))
                                else:
                                    examples = "Something is wrong with your range number, most be positive integer.\n" + examples
                                    await message.channel.send(examples)
                        else:
                            examples = "Something is wrong with your range number, most be positive integer.\n" + examples
                            await message.channel.send(examples)
                    else:#we have 5 inputs
                        num1 = contentArray[2]
                        num2 = contentArray[3]
                        num3 = contentArray[4]
                        trueNum3 = num3
                        if num3[:1] == "_":
                            trueNum3 = num3[1:]
                        if num1.isdigit() and num2.isdigit() and trueNum3.isdigit():
                            await message.channel.send(getProbText(num1, num2, pzlName, trueNum3))
                        else:
                            examples = "Something is wrong with your range number, most be positive integer.\n" + examples
                            await message.channel.send(examples)
                else:
                    examples = "Something is wrong with your puzzle size (most be 4x4 or 3x3)\n" + examples
                    await message.channel.send(examples)
            #getProbText(f1, f2, pzlName, nscr)
        except:
            await message.channel.send("Something is wrong\n```" + traceback.format_exc() + "```")
    if '!ip' in message.content.lower():
        try:
            fp = urllib.request.urlopen("https://2ip.ru/")
            mybytes = fp.read()
            mystr = mybytes.decode("utf8")
            mystr = html2text.html2text(mystr)
            mystr = mystr.splitlines()
            fp.close()
            username = "Ваш IP адрес"
            matching = [s for s in mystr if username in s]
            #my_string = ';\n'.join(matching)
            await message.channel.send(mystr[mystr.index(matching[0])+2].replace("__",""))
        except:
            await message.channel.send("Something is wrong\n```"+traceback.format_exc()+"```")
    if 'scrable' in message.content.lower():  
        await message.channel.send("Infinity tps, "+ message.author.mention+"?")
        await message.add_reaction('0️⃣')    
    if '!solve' in message.content.lower():
        try:
            solved_state = slide_solved_state(4)
            neighbours = slide_neighbours(4)
            is_goal = lambda p: p == solved_state
            thingy = message.content[7:]
            arr = tuple([int(a) for x in thingy.split("/") for a in x.split()])
            print(arr)
            tic = time.perf_counter()
            #arr = (15, 14, 1, 6, 9, 11, 4, 12, 0, 10, 7, 3, 13, 8, 5, 2)
            slide_solver = IDAStar(slide_wd(4, solved_state), neighbours)
            path, moves, cost, num_eval = slide_solver.solve(arr, is_goal, 80)
            toc = time.perf_counter()
            dif = (f"Elapsed time: {toc - tic:0.2f} seconds")
            #slide_print(arr)
            try:
                sol="".join({-1: "R", 1: "L", -4: "D", 4: "U"}[move[1]] for move in moves)
                await message.channel.send("Solution for "+thingy+ "\n"+sol+"\n"+str(cost)+" moves\n"+dif)
            except Exception as e:
                await message.channel.send(str(e)+"\nProbably your scramble is too hard for me :(\n"+dif)
            #print(cost)     
        except Exception as e:
            await message.channel.send("Something is wrong\n```"+str(e)+"```")
    if '!help' in message.content.lower():
        await message.channel.send("Egg bot commands:```\n- !getwr <NxM> - get wr for this puzzle (single)\n- !wrsby <username> - get all wrs for that username [admin for big message]\n- !ip - get ip for multiplayer server (command works all the time even if server is offline, ask me for hosting)\n- !solve <scramble> - solve some 4x4 scramble\n- !getpb <user> <puzzle> - get pb for one of 30 main categories in tier ranks, puzzle for 3x3 to 10x10\n- !getreq <tier> <puzzle> - get requirement for getting tier in new ranked system\n- !getprob <puzzle> <moves> [<moves>, <amount> || <amount] - get probability of getting N moves optimal scramble, type !getprob for examples```")
@tasks.loop(seconds=1)
async def spam(chan, msg):
    await chan.send(msg)
client.run('YOURSECRETKEY') #KEYNEEDED


 
 
 
