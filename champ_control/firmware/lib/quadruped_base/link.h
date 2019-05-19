class Link
{
    public:
        float d, theta, r, alpha;

        Link(float _d, float _theta, float _r, float _alpha) : d(_d), theta(_theta), r(_r), alpha(_alpha) { }
        virtual void Move(float amount) = 0;
};