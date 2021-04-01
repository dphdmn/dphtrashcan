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
from itertools import count
from multiprocessing import Process
from array import *


client = discord.Client()

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
        await message.channel.send("Egg bot commands:```\n- !getwr <NxM> - get wr for this puzzle (single)\n- !wrsby <username> - get all wrs for that username [admin for big message]\n- !ip - get ip for multiplayer server (command works all the time even if server is offline, ask me for hosting)\n- !solve <scramble> - solve some 4x4 scramble```")
@tasks.loop(seconds=1)
async def spam(chan, msg):
    await chan.send(msg)
client.run('PASTSECRETKEYHERE') //PASTE CODE HERE


 
 
 
