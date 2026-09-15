# **BOOLE’S PRINCIPLES OF SYMBOLICAL REASONING** 

STANLEY BURRIS AND H.P. SANKAPPANAVAR 

Abstract. In modern algebra it is well-known that one cannot, in general, apply ordinary equational reasoning when dealing with partial algebras. However Boole did not know this, and he took the opposite to be a fundamental truth, which he called the Principles of Symbolical Reasoning in his 1854 book _Laws of Thought_ . Although Boole made no mention of it, his Principles were clearly a generalization of the earlier publications on algebra by the Cambridge mathematician Peacock. After a detailed examination of Boole’s presentation of his Principles, we give a correct version that is applicable to his algebra of logic for classes. 

When Boole started his mathematical research in the late 1830s, he was particularly fascinated by two tools recently introduced into mathematics, the use of _operators_ (to solve differential equations), and the _symbolical method_ (which justified algebraic derivations even though the intermediate steps were not interpretable). Differential operators had been introduced by the French and used by prominent French mathematicians such as Cauchy; but by 1830 the French had pretty much abandoned them—Cauchy felt that not enough was known about them to ensure that their use led to correct results. 

English mathematicians, especially Boole, took up differential operators with great enthusiasm, and in 1844 he won a gold medal from the Royal Society for a paper based on them. When he turned to create his algebra of logic for classes in 1847, it was natural for Boole to build it on a foundation of operators, namely selection operators. For example, the expression “big shiny ” was viewed as the operator y that selected shiny objects from a class, followed by the operator _x_ that selected “big” objects. The composition of the two operators was written simply as _x_ y, and, as with differential operators, _x_ y was viewed as the product of _x_ and y. It only remained for Boole to find appropriate definitions for addition and subtraction. In the 1854 version of his algebra of logic, Boole replaced the symbols for selection operators with the symbols for classes, relegating the selection operators to a footnote on operations of the mind. 

The symbolical method was introduced by Peacock [12] in his 1830 _Treatise on Algebra_ — this book gave his resolution of concerns about negative and complex numbers. (The _Treatise_ was expanded into two volumes in 1842/1845.) He split algebra into two parts: (1) _Arithmetical Algebra_ , which was the algebra of positive numbers (which meant the operation of subtraction was a partial operation), and (2) _Symbolical Algebra_ , which dispensed with interpretations and dealt solely with equations that could be derived from basic laws. 

The two kinds of algebra were tied together by Peacock’s fundamental principle of _The Permanence of Equivalent Forms_ , which basically said that general truths of the Arithmetical Algebra provided the laws of Symbolical Algebra, and equations derived from the laws in the latter gave true facts about positive numbers whenever they applied to them. Arithmetical Algebra had a partial algebra as its interpretation, whereas Symbolical Algebra simply carried out derivations as though the operations were total, without the benefit 

_Date_ : December 10, 2014. 

1 

2 

STANLEY BURRIS AND H.P. SANKAPPANAVAR 

of an interpretation. Thus ?´1 was no longer a mysterious number whose nature was to be debated—it was just a symbolical expression devoid of meaning. 

Boole was able to create an algebra of logic for classes that had laws remarkably close to those of ordinary algebra;<sup>1</sup> but this came at a cost, namely his operations of ` and ´ were partial operations. Since he wanted to freely use algebraic reasoning that was not impeded by the possibility that terms in his derivations might not be interpretable (as classes), he formulated a general version of Peacock’s approach to algebra. Boole’s _Principles of Symbolical Reasoning_ lifted Peacock’s work out of the confines of numbers to the general setting of partial algebras. Briefly stated, Boole’s Principles said that given a collection P of partial algebras and a collection Σ of laws that P satisfied (that is, satisfied in all instances for which the terms in the laws were defined), one could carry out equational reasoning as though one were working with total algebras to obtain valid results for the class P. 

Boole’s Principles are clearly far too general. Here is a simple example to show that the equational logic for total algebras (see [4]) need not be valid for partial algebras. Let **P** “ xt0, 1u, `y be the partial algebra given by 0 ` 0 “ 0 and 1 ` 1 “ 1; and otherwise ` is undefined. Then the equations _x_ ` y « _x_ and _x_ ` y « y hold in **P** whenever _x_ ` y is defined. But _x_ « y, an equational logic consequence of these two equations, does not hold in **P** . 

Boole’s Principles were presented in Chapter V of his 1854 book _Laws of Thought_ (this book will be referred to as LT in this paper). His somewhat rambling introduction to these Principles first started to come into focus with the statement of them on p. 67; his final analysis came on p. 69, where he said that to justify his Principles one only needed a single example.<sup>2</sup> The only example (besides his algebra of logic for classes) that he mentioned with regard to his Principles was that of the trigonometric identities derived using the imaginary number ?´1. Evidently Boole was not convinced that this was the desired single example since he also said that such derivations of trigonometric identities could evidently only be justified by appealing to his Principles. He rounded out the defence of his Principles by saying that they should simply be accepted as _fundamental facts_ about knowledge. 

In §1 below, the long-neglected 1854 discussion by Boole of his Principles is carefully examined. §2 gives a corrected version (Theorem 2.4) of these Principles, based on universal Horn sentences with relativized quantifiers. §3 shows that Theorem 2.4 does indeed apply to Boole’s algebra of logic for classes, thanks to the ground-breaking book of Hailperin [10] on Boole’s work, and to Gorbunov’s analysis of quasi-identity logic [8].<sup>3</sup> 

# 1. An analysis of Boole’s text on his Principles 

Boole claimed, in LT , that the symbols of common algebra were the natural ones to use for an algebra of logic for classes, and it was a happy coincidence that the laws of common algebra agreed with the laws of his algebra of logic.<sup>4</sup> Boole’s algebras **P** p _U_ q “ 

1 As strange as Boole’s use of partial algebras might look to the uninitiated, it seems quite possible that he may have found the simplest way to incorporate the laws he used (stated in §1 below) into an algebra of logic for classes. 

2This was based on his claim that in logic, unlike in the natural sciences, one only needed a single example to justify a general result. 

3We are indebted to our colleagues, Professors Kira Adaricheva and Anvar Nurakunov, for this reference, and to Professor George McNulty for discussions on this topic. 

4 In 1999 Priest [14] noted that the publication of a significant portion of Boole’s Nachlass in [9] clarified the fact that “Boole’s driving inspiration was the analogy between arithmetic and logic”. It is easier to sort out 

BOOLE’S PRINCIPLES 

3 

x _P_ p _U_ q, `, ¨, ´, 0, 1y,where _P_ p _U_ q is the collection of subsets of the universe _U_ , had 0 :“ Ø, 1 :“ _U_ , and multiplication defined as intersection; both addition and subtraction were only partially defined. Indeed, _A_ ` _B_ :“ _A_ Y _B_ and _A_ ´ _B_ :“ _A_ z _B_ , when defined, where the domains of addition and subtraction were given by: 



Since some of the operations of Boole’s algebras were only partially defined, Boole was clearly working with partial algebras. 

Boole found laws of the **P** p _U_ q, such as _x_ ` y « y ` _x_ , and _x_<sup>2</sup> « _x_ , by looking at the instances where the terms of the laws were defined. After a careful study of Boole’s LT , Hailperin [10] determined that the collection of laws actually used by Boole was 

- ‚ NCR 1, the usual laws for commutative rings with unity, and laws excluding additively nilpotent elements ( _nx_ « 0 ñ _x_ « 0, for _n_ a non-zero integer), 

- ‚ 0 � 1, and 

- ‚ the special law _x_<sup>2</sup> « _x_ , which was only applied to class-symbols _A_ , _B_ , . . .. 

In developing his algebra of logic for classes, Boole reasoned with these laws as if the operations were total, thus using reasoning which, in general, is not valid for partial algebras. He would start with ground premiss equations whose terms were defined for all values of the class-symbols, and end up with conclusion equation(s) that likewise had terms that were totally defined. But in the intermediate steps in a derivation of the conclusion(s) he could use equations that had terms that were uninterpretable, that is, only partially defined. This was likely the major stumbling block to the understanding of his algebra of logic by mathematicians interested in working with a symbolic logic. 

Boole presented his defence of the use of uninterpretables in Chapter V of LT , which is titled 

Of the fundamental principles of symbolical reasoning, and of the expansion or development of expressions involving logical symbols. 

The first half of the title is the subject of the first six items of Chapter V, where Boole attempted to allay the reader’s concerns about the legitimacy of using uninterpretables in the intermediate steps of a derivation. These six items are carefully examined, one in each of the the following six subsections, to see to what extent Boole succeeded—the quotes from Chapter V of LT given below are presented as indented paragraphs in sans serif font. 

1.1. **On item 1 of Chap. V of** LT **.** In the first item of Chapter V Boole said that so far he had set up the notation for an algebra of logic, described the fundamental operations that he would use, determined the laws, and had shown how to render primary propositions (his version of categorical propositions) about classes as equations. Next he wanted to develop his algebra of logic by proving theorems and finding algorithms; but first he needed to say something about how he was going to do this. Actually the reader would have to wait till item 4 of Chapter V before learning that Boole would simply be using ordinary equational reasoning on his partial algebras. 

1. The previous chapters of this work have been devoted to the investigation of the fundamental laws of the operations of the mind in reasoning; of their development in the laws of the symbols of Logic; and 

the text of LT if one assumes that Boole started with the goal of using common algebra, with its laws and rules of inference, and shoe-horned the logic of classes into this framework (with the help of a restricted use of the idempotent law). 

4 

STANLEY BURRIS AND H.P. SANKAPPANAVAR 

of the principles of expression, by which that species of propositions called primary may be represented in the language of symbols. These inquiries have been in the strictest sense preliminary. They form an indispensable introduction to one of the chief objects of this treatise— the construction of a system or method of Logic upon the basis of an exact summary of the fundamental laws of thought. There are certain considerations touching the nature of this end, and the means of its attainment, to which I deem it necessary here to direct attention. 

1.2. **On item 2 of Chap. V of** LT **.** In the second item Boole started by saying that in order to have a general method for dealing with the logic of classes he needed general laws and rules of inference. He noted that addition was a partial operation (which presumably could be an obstacle to creating a general method). 

2. I would remark in the first place that the generality of a method in Logic must very much depend upon the generality of its elementary processes and laws. one has, for instance, in the previous sections of this work investigated, among other things, the laws of that logical process of addition which is symbolized by the sign ` . Now those laws have been determined from the study of instances, in all of which it has been a necessary condition, that the classes or things added together in thought should be mutually exclusive. The expression _x_ ` y seems indeed uninterpretable, unless it be assumed that the things represented by _x_ and the things represented by y are entirely separate; that they embrace no individuals in common. 

Boole never satisfactorily explained in his publications why it was necessary to restrict addition _A_ ` _B_ to disjoint classes _A_ , _B_ . The real reason for this restriction on the definition of addition was surely Boole’s desire to use ordinary algebra for an algebra of logic (which he was able to do by adding restricted use of the idempotent law). Then, if _A_ ` _B_ were defined it would be idempotent, that is, p _A_ ` _B_ q<sup>2</sup> “ _A_ ` _B_ . Ordinary algebra, along with the idempotence of _A_ and _B_ , would lead to 2 _AB_ “ 0, and thus to _AB_ “ 0; this meant _A_ and _B_ were disjoint. Boole actually gave the details of deriving _AB_ “ 0 from p _A_ ` _B_ q<sup>2</sup> “ _A_ ` _B_ in his unpublished manuscripts—see p. 21 of [2]. 

And conditions analogous to this have been involved in those acts of conception from the study of which the laws of the other symbolical operations have been ascertained. 

The only other operation that was restricted was subtraction. The (unstated) reason for this restriction is again determined by looking at the consequences of _A_ ´ _B_ being idempotent, as Boole also noted in the aforementioned unpublished manuscript. Next Boole stated the potential Achilles heel of his presentation. 

The question then arises, whether it is necessary to restrict the application of these symbolical laws and processes by the same conditions of interpretability under which the knowledge of them was obtained. 

Boole proceeded to try to make the case that the answer was always “no restrictions needed”, saying that otherwise his program for an algebra of logic must fail. 

If such restriction is necessary, it is manifest that no such thing as a general method in Logic is possible. 

5 

BOOLE’S PRINCIPLES 

By a ‘general method’ Boole meant algorithms to deduce desired equational consequences from equational premisses. Actually it was _not_ manifest that a general method would be impossible under the restriction that all steps only use interpretable terms; it was just that a general method would, in some cases, under such restrictions, be rather unwieldy compared to the method Boole presented. His answer as to whether or not such a restriction was necessary would wait till the end of item 3 below. 

On the other hand, if such restriction is unnecessary, in what light are we to contemplate processes which appear to be uninterpretable in that sphere of thought which they are designed to aid? These questions do not belong to the science of Logic alone. They are equally pertinent to every developed form of human reasoning which is based upon the employment of a symbolical language. 

1.3. **On item 3 of Chap. V of** LT **.** Boole started this item by noting that in everyday reasoning one did not employ uninterpretable steps. 

3. I would observe in the second place, that this apparent failure of correspondency between process and interpretation does not manifest itself in the ordinary applications of human reason. For no operations are there performed of which the meaning and the application are not seen; and to most minds it does not suffice that merely formal reasoning should connect their premises and their conclusions; but every step of the connecting train, every mediate result which is established in the course of demonstration, must be intelligible also. And without doubt, this is both an actual condition and an important safeguard, in the reasonings and discourses of common life. 

Next he said that there are perhaps those who would like to apply the same requirement, of every step being meaningful, to symbolical arguments. Eventually he would claim, in item 4, that it was enough that the premisses and conclusion were meaningful in order to obtain a valid, meaningful argument. 

There are perhaps many who would be disposed to extend the same principle to the general use of symbolical language as an instrument of reasoning. It might be argued, that as the laws or axioms which govern the use of symbols are established upon an investigation of those cases only in which interpretation is possible, one has no right to extend their application to other cases in which interpretation is impossible or doubtful, even though (as should be admitted) such application is employed in the intermediate steps of demonstration only. 

Next he repeated his belief that the symbolical method offered little to the study of logic unless one could use the laws of the partial algebras freely. 

Were this objection conclusive, it must be acknowledged that slight advantage would accrue from the use of a symbolical method in Logic. Perhaps that advantage would be confined to the mechanical gain of employing short and convenient symbols in the place of more cumbrous ones. 

The phrase ‘it must be acknowledged’really meant that Boole believed there would be little gain in developing an algebra of logic if one were required to make terms interpretable in every step. At this point the reader would be justified in expecting Boole to offer a profound 

6 

STANLEY BURRIS AND H.P. SANKAPPANAVAR 

insight concerning uninterpretable steps in symbolical reasoning. Instead one simply hears the voice of authority. 

But the objection itself is fallacious. Whatever our `a priori anticipations might be, it is an unquestionable fact that the validity of a conclusion arrived at by any symbolical process of reasoning, does not depend upon our ability to interpret the formal results which have presented themselves in the different stages of the investigation. 

The assertion Boole makes about ‘an unquestionable fact’ is wrong. It is simply an erroneous belief that Boole firmly held; his algebra of logic would need to be completely reworked without this ‘fact’. 

There exist, in fact, certain general principles relating to the use of symbolical methods, which, as pertaining to the particular subject of Logic, I shall first state, and I shall then offer some remarks upon the nature and upon the grounds of their claim to acceptance. 

The reader will search in vain for a clear statement of any actual grounds for acceptance of the Principles in Boole’s text. It seems that the use of the Principles to prove general results about complex numbers and about logic, results which were sound in all the examples Boole had checked, were his only grounds. 

1.4. **On item 4 of Chap. V of** LT **.** In this item Boole laid out the requirements of symbolical reasoning that he believed were sufficient to guarantee the validity of the results whenever they were interpretable. 

4. The conditions of valid reasoning, by the aid of symbols, are— 

1st, That a fixed interpretation be assigned to the symbols employed in the expression of the data; and that the laws of the combination of those symbols be correctly determined from that interpretation. 

The first condition said that one was to work with a fixed collection of partial algebras, and that the laws one worked with actually held whenever the terms of the laws were defined in the partial algebras. 

2nd, That the formal processes of solution or demonstration be conducted throughout in obedience to all the laws determined as above, without regard to the question of the interpretability of the particular results obtained. 

Clearly the part of the 2nd requirement that said ‘don’t worry about interpretability’ is the contentious point in Boole’s conditions; he would have more to say about this in item 5 of Chap. V. 

3rd, That the final result be interpretable in form, and that it be actually interpreted in accordance with that system of interpretation which has been employed in the expression of the data. Concerning these principles, the following observations may be made. 

The 3rd condition said that the conclusion needed to be interpretable (so that one had a meaningful/useful result). 

1.5. **On item 5 of Chap. V of** LT **.** Here Boole reflected on the naturalness of the first and third condition, but noted that the second condition likely needed “a few additional words”. 

5. The necessity of a fixed interpretation of the symbols has already been sufficiently dwelt upon (II. 3). The necessity that the fixed result 

BOOLE’S PRINCIPLES 

7 

should be in such a form as to admit of that interpretation being applied, is founded on the obvious principle, that the use of symbols is a means towards an end, that end being the knowledge of some intelligible fact or truth. And that this end may be attained, the final result which expresses the symbolical conclusion must be in an interpretable form. It is, however, in connexion with the second of the above general principles or conditions (V. 4), that the greatest difficulty is likely to be felt, and upon this point a few additional words are necessary. 

What followed was a somewhat confused attempt by Boole to justify his Principles. He said they rested on another fact that he had become aware of—that whereas the natural sciences required many observations to deduce a law of nature, in logic it was different. He said that a single clear example made the general principle known. 

I would then remark, that the principle in question may be considered as resting upon a general law of the mind, the knowledge of which is not given to us `a priori, i.e. antecedently to experience, but is derived, like the knowledge of the other laws of the mind, from the clear manifestation of the general principle in the particular instance. 

No evidence for this sweeping claim was provided, just a continuation of the claim. In reality, fundamental principles of reasoning are based on their acceptance by the community of scholars. If Boole had only said that he was proposing that his Principles be accepted as fundamental, his writing style in these sections would have been more agreeable. 

A single example of reasoning, in which symbols are employed in obedience to laws founded upon their interpretation, but without any sustained reference to that interpretation, the chain of demonstration conducting us through intermediate steps which are not interpretable, to a final result which is interpretable, seems not only to establish the validity of the particular application, but to make known to us the general law manifested therein. No accumulation of instances can properly add weight to such evidence. It may furnish us with clearer conceptions of that common element of truth upon which the application of the principle depends, and so prepare the way for its reception. It may, where the immediate force of the evidence is not felt, serve as a verification, `a posteriori, of the practical validity of the principle in question. But this does not affect the position affirmed, viz., that the general principle must be seen in the particular instance,—seen to be general in application as well as true in the special example. 

Now it seemed that Boole was ready for the coup de grˆace, to give that single example to show that, according to his ‘one example is enough’ thesis, his Principles were sound. He stated what is likely the only example he knew of where mathematicians worked with uninterpretables, namely the use of the uninterpretable ?´1 when working with numbers. As an interesting example of results obtained by using ?´1 he mentioned trigonometric identities.<sup>5</sup> He started by saying this was an example of what had been said—unfortunately 

5 This likely refered to results such as DeMoivre’s Theorem, that 

pcos θ ` _i_ sin θq<sup>_n_</sup> “ cosp _n_ θq ` _i_ sinp _n_ θq. 

This example would have been quite difficult for Boole to set up for his Principles, presumably starting with an algebra on the reals. What would the fundamental operations be that allowed one to discuss cos and sin? and what would the laws be that led to a proof of DeMoivre’s Theorem? 

8 

STANLEY BURRIS AND H.P. SANKAPPANAVAR 

it does not seem to be an example for ‘one example is enough’, but rather just another application of his Principles. 

The employment of the uninterpretable symbol ?´1 , in the intermediate processes of trigonometry, furnishes an illustration of what has been said. I apprehend that there is no mode of explaining that application which does not covertly assume the very principle in question. 

Thus Boole ended up not offering the single example to justify his Principles; instead he offered two examples, common algebra and his algebra of logic, where he believed his Principles applied.<sup>6</sup> Next one sees Boole simply claiming that his Principles deserved to be accepted as fundamental facts. He could have replaced the totality of items 1–5 in Chap. V with simply stating his Principles and making the next statement, leaving it to the reader to decide whether or not to accept them. 

But that principle, though not, as I conceive, warranted by formal reasoning based upon other grounds, seems to deserve a place among those axiomatic truths which constitute, in some sense, the foundation of the possibility of general knowledge, and which may properly 

be regarded as expressions of the mind’s own laws and constitution. 

Of course Boole’s Principles did not take hold; they seem to have quietly vanished. Many mathematicians and logicians, starting with Cayley and Jevons, did not like Boole’s uninterpretables. However we are not aware of any objections prior to 1999 to Boole’s belief that “one example is enough” to justify his Principles. In 1999 Priest [14] wrote that “Even in logic, the truth of a general rule cannot be simply read off from a particular case”. We are not aware of any publication that has addressed the issues with Boole’s Principles. Mathematicians, following Jevons and Peirce, soon side-stepped the issues by modifying Boole’s algebra of logic, replacing Boole’s addition by union and his subtraction by complement, so that there were no uninterpretables. 

1.6. **On item 6 of Chap. V of** LT **.** This item of LT offers more puzzling comments by Boole. He said that the Principles would be used in LT in the following manner: when carrying out an argument in his algebra for the logic of classes, if one encountered a step that had uninterpretable terms then one was to stop thinking about classes and switch to thinking about the step as applying to the algebra of 0 and 1 described in his Rule of 0 and 1.<sup>7</sup> When one eventually returned to steps where the terms were interpretable in the logic of classes, then one switched back to thinking about classes. 

It seems the only reason for using the algebra of 0 and 1 for the steps with uninterpretable terms in the logic of classes was to help the user remember the laws and procedures that one could use. Otherwise it played no role. 

6. The following is the mode in which the principle above stated will be applied in the present work. It has been seen, that any system of propositions may be expressed by equations involving symbols _x_ , y , _z_ , which, whenever interpretation is possible, are subject to laws identical in form with the laws of a system of quantitative symbols, susceptible only of the values 0 and 1 (II. 15). But as the formal processes of 

> 6 One wonders why Boole bothered bringing up his ‘single example suffices’ assertion—was he hoping that someone else would provide the one example needed? or perhaps that the mathematical community would say that the application of complex numbers to trigonometric identities was valid without reference to his Principles, and hence could be used as the one example? 

> 7 This rule is described in detail in [3]. 

BOOLE’S PRINCIPLES 

9 

reasoning depend only upon the laws of the symbols, and not upon the nature of their interpretation, we are permitted to treat the above symbols, _x_ , y , _z_ , as if they were quantitative symbols of the kind above described. We may in fact lay aside the logical interpretation of the symbols in the given equation; convert them into quantitative symbols, susceptible only of the values 0 and 1 ; perform upon them as such all the requisite processes of solution; and finally restore to them their logical interpretation. And this is the mode of procedure which will actually be adopted, though it will be deemed unnecessary to restate in every instance the nature of the transformation employed. 

Next Boole reminded us, again, of how important he thought it was to be able to work as though one had a total algebra, for he believed that otherwise the quest for the desired algorithms (to derive certain kinds of conclusions) would be hopeless. 

The processes to which the symbols _x_ , y , _z_ , regarded as quantitative and of the species above described, are subject, are not limited by those conditions of thought to which they would, if performed upon purely logical symbols, be subject, and a freedom of operation is given to us in the use of them, without which, the inquiry after a general method in Logic would be a hopeless quest. 

Boole concluded this section by saying he had a general method to convert any equational conclusion into an equivalent one that was interpretable, and that would be presented next. 

Now the above system of processes would conduct us to no intelligible result, unless the final equations resulting therefrom were in a form which should render their interpretation, after restoring to the symbols their logical significance, possible. There exists, however, a general method of reducing equations to such a form, and the remainder of this chapter will be devoted to its consideration. 

This general method was based on Boole’s Expansion Theorem, an analog of the disjunctive normal form used in modern Boolean algebra (see [3] for details). 

2. Correcting Boole’s Principles 

Throughout this section it is assumed that: (i) all partial algebras being discussed belong to a fixed language F ; (ii) partial algebras are denoted by capital bold letters **P** , **Q** , etc. (iii) **x** is the list _x_ 1, . . ., _xm_ , and **A** is the list _A_ 1, . . . , _Am_ ; (iv) p@ **x** q means p@ _x_ 1q ¨ ¨ ¨ p@ _xm_ q; (v) _t_ :“ _t_ p **x** q denotes a term. 

Given a partial algebra **P** , a term _t_ defines a partial operation _t_<sup>**P**</sup> on **P** with domain Dom p _t_<sup>**P**</sup> q. The domain Dom **P** pωq of an open formula ωp **x** q is the intersection of the Dom p _t_<sup>**P**</sup> q, for _t_ a term appearing in ω. The relation ω<sup>**P**</sup> defined by ω is the collection of **p** P Dom **P** pωq such that ωp **p** q is true in **P** . The following notion of subalgebra will be used when working with partial algebras. 

**Definition 2.1.** _Define_ **P** Ď **Q** _if P_ Ď _Q and the operations of_ **P** _, where defined, agree with those of_ **Q** _, that is_ 

(1) _f_<sup>**P**</sup> p **p** q “ _p implies f_<sup>**Q**</sup> p **p** q “ _p_ , 

_for_ **p** P Dom p _f_<sup>**P**</sup> q _._ 

One easily sees that the following hold. 

10 

STANLEY BURRIS AND H.P. SANKAPPANAVAR 

**Lemma 2.2.** _Suppose_ **P** Ď **Q** _._ 

**(a)** _If_ **p** P Dom p _t_<sup>**P**</sup> q _then_ **p** P Dom p _t_<sup>**Q**</sup> q _and t_<sup>**Q**</sup> p **p** q “ _t_<sup>**P**</sup> p **p** q. **(b)** _For_ ωp **x** q _an open formula with_ **p** P Dom **P** pωq _, one has_ **p** P Dom **Q** pωq _and_ (2) **P** |ù ωp **p** q _i_ ff **Q** |ù ωp **p** q. **Definition 2.3.** _Given two partial algebras_ **P** _and_ **Q** _, a mapping_ α : _P_ Ñ _Q is an embedding of_ **P** _into_ **Q** _i_ ff α _is 1-1 and for each fundamental operation f, if_ **p** P Dom p _f_<sup>**P**</sup> q _then_ αp **p** q P Dom p _f_<sup>**Q**</sup> q _and_ 



Let @ Horn be the set of universal Horn sentences belonging to the given language of partial algebras, let Σ be a subset of @ Horn , let δp _x_ q be a conjunction of atomic formulas, and let P be a collection of partial algebras **P** “ x _P_ , F y. The following conventions are adopted, where σ P @ Horn , say σ :“ p@ **x** qωp **x** q with ω :“ ωp **x** q an open Horn formula: 

**(a)** Whenever a property Π of partial algebras **P** is stated for P, this means it applies to all members of P. **(b)** ω is **P** -total if Dom **P** pωq “ _P_<sup>_m_</sup> , and it is P-total if it is total for all **P** P P. Define Dom **P** pσq :“ Dom **P** pωq, etc. **(c)** σ holds in a partial algebra **P** if ωp **p** q is true in **P** for all **p** P Dom pσq. If so, we also say **P** satisfies σ, abbreviated as **P** |ù σ. **P** |ù Σ means **P** |ù σ for σ P Σ. P |ù σ if **P** |ù Σ for all **P** P P. **(d)** A law of P is any σ such that P |ù σ. **(e)** σ|δ is the universal Horn sentence obtained by relativizing the quantifiers of σ to δp _x_ q. **(f)** Mod pΣq is the collection of total algebras **P** satisfying Σ. **(g) P** embeds in Mod pΣq, written **P** ãÑ Mod pΣq, if there is a **Q** P Mod pΣq and an embedding α : **P** ãÑ **Q** . P ãÑ Mod pΣq if **P** ãÑ Mod pΣq for every **P** P P. **(h)** Σ $ σ means there is a derivation of σ from Σ in first-order logic. **(i)** Diag<sup>`</sup> p **P** q is the set of formulas _f_ p **p** q « _p_ , where **p** P Dom p _f_<sup>**P**</sup> q and _p_ P _P_ are such that _f_<sup>**P**</sup> p **p** q “ _p_ . (This describes the tables for the fundamental operations of **P** .) With this notation Boole’s Principles can be stated as:<sup>8</sup> 

_If_ P |ù Σ Y tp@ _x_ qδp _x_ qu _and_ δp _x_ q _is_ P _-total then_ (3) p@σ P @ Horn q`Σ $ σ|δ ñ P |ù σq. Example 3.1 shows that this is false in general. The correct version is given in the next theorem, and §3 shows how it applies to Boole’s algebra of logic for classes. 

**Theorem 2.4.** _Suppose_ ΣYtp@ _x_ qδp _x_ qu _is a set of laws for_ P _with_ δp _x_ q _a_ P _-total formula. Then_ 

(4) p@σ P @ Horn q`Σ $ σ|δ ñ P |ù σq _i_ ff (5) P ãÑ Mod pΣq. 

8 Boole’s formulation of his Principles was incomplete for the application he had in mind, namely to his algebra of logic for classes. He did not discuss the possibility of restricting some of the laws so that they applied only to the class symbols, a restriction he quietly imposed on the idempotent law _x_<sup>2</sup> « _x_ . This restriction is handled by our δp _x_ q. 

BOOLE’S PRINCIPLES 

11 

_Proof._ (ð) Suppose that (5) holds and Σ $ σ|δ. Given **P** P P let **P** ãÑ **Q** P Mod pΣq. From **Q** P Mod pΣq it follows that **Q** |ù σ|δ. Since σ|δ is a universal sentence, it follows that **P** |ù σ|δ. Now δ is **P** -total and **P** |ù p@ _x_ qδp _x_ q, thus **P** |ù σ. 

(ñ) For the converse suppose that (5) fails. Then, for some **P** P P, 

(6) Σ Y Diag<sup>`</sup> p **P** q Y Distinct p _P_ q 

is inconsistent, where Distinct p _P_ q is the set of negated atomic formulas _p_ � _q_ for pairs _p_ , _q_ of distinct elements of _P_ . By compactness there is a finite _P_ 0 :“ t _p_ 1, . . ., _pm_ u Ď _P_ and a finite subset S of Diag<sup>`</sup> p **P** q, with the only _p_ P _P_ mentioned in S being members of _P_ 0, such that 

(7) Σ Y S Y Distinct p _P_ 0q 

is inconsistent. Since this is a set of Horn sentences, for some _i_ ă _j_ , say _i_ “ 1 and _j_ “ 2, we have 

(8) Σ Y S Y t _p_ 1 � _p_ 2u 

is inconsistent. This is equivalent to 



We can assume, w.l.o.g, that δp _p_ q P S for _p_ P _P_ 0. Thus we can write (9) in the form 



where Ωp **p** q is a conjunction of atomic sentences from S . This implies 



But **P** ̸ |ù σ since **p** P Dom **P** pσq, but it fails to make the matrix of σ, namely 



true. Thus the assumption that (5) fails leads to the fact that (4) fails. 

□ 

One can ask if there is a parallel result when working with identities. 

**Question 1.** _Suppose_ P |ù Σ _where_ Σ _is a set of identities. Does one have_ 

(13) p@σ P Identities q`Σ $ σ ñ P |ù σq _i_ ff (14) P ãÑ Mod pΣq ? 

12 

STANLEY BURRIS AND H.P. SANKAPPANAVAR 

# 3. Application to Boole’s algebra of logic for classes 

Recall that Boole defined a partial algebra **P** _U_ on the collection of subclasses of a universe _U_ � Ø as follows:<sup>9</sup> 



Let P be the collection of **P** _U_ . 

As mentioned earlier, Boole used, for the laws of P, the usual equational laws _CR_ 1 for commutative rings with unity along with the law 0 � 1 and the quasi-identity laws that express ‘no additively nilpotent elements’. This set of universal Horn sentences will be denoted by Σ. Boole also used the idempotent law _x_<sup>2</sup> « _x_ , but in a limited sense—it only applied to class-symbols. Define δp _x_ q to be _x_<sup>2</sup> « _x_ . 

Note that PãÑ Mod pΣq, namely one can easily embed **P** _U_ into Z<sup>_U_</sup> , the ring of integers raised to the power _U_ , by mapping _A_ Ď _U_ to its characteristic function χ _A_ . In view of Theorem 2.4 it is not surprising that Hailperin used this fact to show that a large portion of Boole’s algebra of logic for classes could be put on a firm basis—one has, from Theorem 2.4, 

(15) 



Boole was interested in applying usual equational reasoning, based on the laws _CR_ 1 of numbers, to justify certain ground equational arguments: 



He used the no-nilpotent-elements laws as inference rules (e.g., from 2 _t_ p **A** q « 0 one has _t_ p **A** q « 0). The idempotent law only applied to the symbols _Ai_ . Let 



To justify (16) using Boole’s ground equational reasoning is equivalent to showing Σ $ σ|δ; see Corollary 2.2.4 in Gorbunov [8]. 

So suppose (16) follows from Boole’s ground equational reasoning. Then Σ $ σ|δ, so by (15), P |ù σ. This says (16) is indeed valid in P. 

**Example 3.1.** _It is surprising that Boole formulated his Principles so generally since a counter-example was readily available from what he knew. With_ Σ _as above, let_ Σ1 _be_ ΣYp@ _x_ qp _x_<sup>2</sup> « _x_ q _. Then, with_ P _being Boole’s class of partial algebras, one has_ P |ù Σ1 _. However his Principles fail to hold since the following is false:_ 



9 Note: To be in agreement with Boole’s vocabulary the word ‘class’ is used here where the modern usage would usually be ‘set’. 

BOOLE’S PRINCIPLES 

13 

_To see that_ (17) _is false, note that_ Σ1 $ p@ _x_ q`p2 _x_ q<sup>2</sup> « 2 _x_ ˘ _, leading to_ Σ1 $ p@ _x_ qp2 _x_ « 0q _. Then_ Σ1 $ σ :“ p@ _x_ qp _x_ « 0q _. But clearly this_ σ _is not satisfied by_ P _._ 

# References 

- [1] George Boole, _An Investigation of The Laws of Thought on Which are Founded the Mathematical Theories of Logic and Probabilities_ . Originally published by Macmillan, London, 1854. Reprint by Dover, 1958. 

- [2] George Boole, _Collected Logical Works, Vol. I, Studies in Logic and Probability_ . Open Court, 1952. 

- [3] Stanley Burris, _George Boole_ . The online Stanford Encyclopedia of Philosophy at http://plato.stanford.edu/entries/boole/. 

- [4] S. Burris and H.P. Sankappanavar, _A Course in Universal Algebra_ . Springer Verlag, 1981. The free, corrected version (2012) is available online as a PDF file at math.uwaterloo.ca/ „ snburris . 

- [5] , _The Horn theory of Boole’s partial algebras._ Bull. Symbolic Logic **19** (2013), no. 1, 97–105. 

- [6] , _Boole’s method I. A modern version._ ArXiv preprint. 

- [7] C.F. Gauss, _Theoria residuorum biquadraticorum_ . Commentatio secunda., Comm. Soc. Reg. Sci. G¨ottingen **7** (1832), 1–34; reprinted in _Werke_ , Georg Olms Verlag, Hildesheim, 1973, pp. 93–148. 

- [8] Viktor A. Gorbunov, _Algebraic Theory of Quasivarieties_ . Translated from the Russian. Siberian School of Algebra and Logic. Consultants Bureau, New York, 1998. xii+298 pp. 

- [9] Grattan-Guinness and Bornet (eds.) _George Boole: Selected Manuscripts on Logic and Philosophy,_ Birkh¨auser, Basel-Boston-Berlin, 1997. 

- [10] Theodore Hailperin, _Boole’s Logic and Probability, (Series: Studies in Logic and the Foundations of Mathematics, 85_ , Amsterdam, New York, Oxford: Elsevier North-Holland. 1976. 2nd edition, Revised and enlarged, 1986. 

- [11] William Rowan Hamilton, _Theory of Conjugate Functions, or Algebraic Couples; with a Preliminary and Elementary Essay on Algebra as the Science of Pure Time._ Transactions of the Royal Irish Academy **17** (1837), 293422. 

- [12] George Peacock, _Treatise on Algebra_ . Cambridge, 1830. 2nd ed. 2 vols., Cambridge 1842/1845. 

- [13] , _Report on the Recent Progress and Present State of certain Branches of Analysis_ , presented at the 3rd meeting of the British Association for the Advancement of Science, 1833. 

- [14] Graham Priest, Review of [9], Studia Logica: An International Journal for Symbolic Logic, **63** , No. 1 (Jul., 1999), pp. 143-146. 

Department of Pure Mathematics, University of Waterloo, Waterloo, Ontario, N2L 3G1, Canada _E-mail address_ : snburris@math.uwaterloo.ca 

Department of Mathematics, SUNY at New Paltz, New Paltz, New York 12561, USA _E-mail address_ : sankapph@newpaltz.edu 

