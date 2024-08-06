commport_ss = ss.source_set()
commport_ss.add(files('commport.c'))
system_ss.add_all(when: 'CONFIG_COMMPORT', if_true: commport_ss)