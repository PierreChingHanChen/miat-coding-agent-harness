import multiprocessing as mp
def pmap(fn, items, procs=2):
    with mp.get_context("fork").Pool(procs) as p:
        return p.map(fn, items, chunksize=max(1, len(items)//(procs*8)))
