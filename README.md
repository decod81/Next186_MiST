# Next186_MiST
Modified version of Next186. Original code by Nicolae Dumitrache, port to MiST by Gyorgy Szombathelyi.

This core implements a vsync/hsync fix and improves cache coherence for eliminating a bug that caused graphical glitches in the original core. Additionally, the CPU/SDRAM frequencies have been increased to 70/140 MHz.

Directory /lemmings/ contains modifies files for the game LEMMINGS for Next186 on MiST. This modification bypasses adlib detection which would otherwise fail if the CPU speed is too high. This bug also occurs on real PC so the behavior is faithful to the original.

Directory /supaplex/ contains improved version of the game SUPAPLEX for Next186 on MiST.
