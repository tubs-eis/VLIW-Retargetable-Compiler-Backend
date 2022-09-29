
An operation is encoded with a name, a latency and the bit encoding.
A = target register
B = src1
C = src2
The values with X are dont care
pre defined bits encode the functional unit 
<opp name="ADD">
        <lat value="1" />
        001XX XXXXXXX BBBBBBB CCCCCC AAAAAAA
</opp>
<opp name="SUB">
        <lat value="1" />
        010XX XXXXXXX BBBBBBB CCCCCC AAAAAAA
</opp>