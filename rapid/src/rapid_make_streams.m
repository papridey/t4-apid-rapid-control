function streams = rapid_make_streams(masterSeed)
    streams.truePert = RandStream('mt19937ar', 'Seed', masterSeed + 11);
    streams.meas     = RandStream('mt19937ar', 'Seed', masterSeed + 22);
    streams.scen     = RandStream('mt19937ar', 'Seed', masterSeed + 33);
    streams.costBase = masterSeed + 2000000;
end
