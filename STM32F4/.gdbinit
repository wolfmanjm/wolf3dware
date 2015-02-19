define reload
  kill
  load
end

define freespace
  printf "free space: %d bytes\n", $msp - _sbrk::heap_end
end

define stacksize
    printf "stack size: %d bytes\n", 0x20030000 - (unsigned int)$msp 
end

define heapwalk
    set var $chunk_curr=(unsigned int)&_end
    set var $chunk_number=1
    while ($chunk_curr < _sbrk::heap_end)
        set var $chunk_size=*(unsigned int*)($chunk_curr+4) & ~1
        set var $chunk_next=$chunk_curr + $chunk_size
        set var $chunk_inuse=*(unsigned int*)($chunk_next+4) & 1
        #set var $chunk_tag=*(unsigned int*)$chunk_next
        printf "Allocation: %u  Address: 0x%08X  Size:%u  ", $chunk_number, $chunk_curr+8, $chunk_size-4
        if ($chunk_inuse)
            #info line *($chunk_tag)
        else
            printf "FREE CHUNK\n"
        end
        set var $chunk_curr=$chunk_next
        set var $chunk_number=$chunk_number+1
    end
end

target extended localhost:3333


