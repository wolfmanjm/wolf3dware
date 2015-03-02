define reload
  kill
  load
end

define freespace
  printf "free space: %d bytes\n", $msp - _sbrk::heap_end
end

define heapused
  printf "used heap: %d bytes\n", _sbrk::heap_end - &_end
end

define stacksize
    printf "stack size: %d bytes\n", 0x20030000 - (unsigned int)$msp
end

define heapwalk
    set var $chunk_curr=(unsigned int)&_end
    set var $chunk_number=1
    set var $used_space=0
    set var $free_space=0
    while ($chunk_curr < _sbrk::heap_end)
        set var $chunk_size=*(unsigned int*)($chunk_curr+4) & ~1
        set var $chunk_next=$chunk_curr + $chunk_size
        set var $chunk_inuse=*(unsigned int*)($chunk_next+4) & 1
        printf "Allocation: %u  Address: 0x%08X  Size:%u  ", $chunk_number, $chunk_curr+8, $chunk_size-8
        if ($chunk_inuse)
            #info line *($chunk_tag)
	    set var $used_space=$used_space+$chunk_size-8
            printf "USED\n"
        else
	    set var $free_space=$free_space+$chunk_size-8
            printf "FREE\n"
        end
        set var $chunk_curr=$chunk_next
        set var $chunk_number=$chunk_number+1
    end
    printf "Used malloc: %u bytes\n", $used_space
    printf "Free malloc: %u bytes\n", $free_space
    printf "Used heap: %d bytes\n", _sbrk::heap_end - &_end
    printf "Unused heap: %d bytes\n", $msp - _sbrk::heap_end
    printf "Total free: %d bytes\n", ($msp - _sbrk::heap_end) + $free_space
end

define freememory
    set var $chunk_curr=(unsigned int)&_end
    set var $chunk_number=1
    set var $used_space=0
    set var $free_space=0
    while ($chunk_curr < _sbrk::heap_end)
        set var $chunk_size=*(unsigned int*)($chunk_curr+4) & ~1
        set var $chunk_next=$chunk_curr + $chunk_size
        set var $chunk_inuse=*(unsigned int*)($chunk_next+4) & 1

        if ($chunk_inuse)
	    set var $used_space=$used_space+$chunk_size-8
        else
	    set var $free_space=$free_space+$chunk_size-8
        end
        set var $chunk_curr=$chunk_next
        set var $chunk_number=$chunk_number+1
    end
    printf "Used malloc: %u bytes\n", $used_space
    printf "Free malloc: %u bytes\n", $free_space
    printf "Used heap: %d bytes\n", _sbrk::heap_end - &_end
    printf "Unused heap: %d bytes\n", $msp - _sbrk::heap_end
    printf "Total free: %d bytes\n", ($msp - _sbrk::heap_end) + $free_space
end

target extended localhost:3333


