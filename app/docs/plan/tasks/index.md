# Task Order

Execute in this order unless blocked:

1. T0001
2. T0002
3. T0101
4. T0102
5. T0201
6. T0202
7. T0203
8. T0204
9. T0301
10. T0302
11. T0303
12. T0401
13. T0402
14. T0403
15. T0501
16. T0502
17. T0503
18. T0601
19. T0602
20. T0603
21. T0701
22. T0702
23. T0801
24. T0802
25. T0901
26. T0902
27. T1001
28. T1002

## Dependency Notes

1. P00 tasks must complete before code refactor tasks.
2. P01 must complete before P02.
3. P02 include cutover must complete before P03+.
4. P07 depends on extracted modules from P04-P06.
5. P08 must happen after P07 API composition.

