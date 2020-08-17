#include <sstream>

#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>

#include <columnarinRecord.h>

#include <flame/state/vector.h>
#include "flame.h"

namespace {
namespace pvd = epics::pvData;

struct SimBPM {
    VIOCObserver* last;
    std::string colX, colY;

    pvd::shared_vector<const double> X, Y;
};

struct SimBPMTable : public SimDev
{
    unsigned param_index;
    std::vector<SimBPM> bpms;

    Machine::lookup_iterator start,end;

    pvd::shared_vector<const pvd::uint32> sec, ns;
};

long orbit_init_ci(columnarinRecord *prec)
{
    const char *link = prec->inp.value.instio.string;
    try {
        // "simname elementname"
        // matches 2 groups
        static boost::regex linkpat("(\\S+) ([^\\s\\[]+)");

        boost::cmatch M;
        if(!boost::regex_match(link, M, linkpat))
            throw std::runtime_error("Bad link string");

        flame::auto_ptr<SimBPMTable> priv(new SimBPMTable);
        priv->prec = (dbCommon*)prec;

        if(!find(SimGlobal.sims, M.str(1), priv->sim))
            throw std::runtime_error("No such simulation instance");

        std::pair<Machine::lookup_iterator, Machine::lookup_iterator> range = priv->sim->machine->equal_range_type(M.str(2));

        if(range.first==range.second)
            throw std::runtime_error("No such elements");

        priv->start = range.first;
        priv->end = range.second;

        columnarin::add_column(prec, "secondsPastEpoch", "sec", pvd::pvUInt);
        columnarin::add_column(prec, "nanoseconds", "ns", pvd::pvUInt);

        for(Machine::lookup_iterator it = range.first; it!=range.second; ++it) {
            ElementVoid *elem = *it;
            SimBPM temp;
            temp.colX = temp.colY = elem->name;
            temp.colX += 'X';
            temp.colY += 'Y';

            columnarin::add_column(prec, temp.colX.c_str(), temp.colX.c_str(), pvd::pvDouble);
            columnarin::add_column(prec, temp.colY.c_str(), temp.colY.c_str(), pvd::pvDouble);

            temp.last = priv->sim->get_measure(elem->index);
            priv->bpms.push_back(temp);
        }

        unsigned idx;
        {
            Config empty;
            flame::auto_ptr<StateBase> state(priv->sim->machine->allocState(empty));

            for(idx=0; true; idx++) {
                StateBase::ArrayInfo info;
                if(!state->getArray(idx, info))
                    throw std::runtime_error("state has no parameter");

                if(strcmp(info.name, "state")==0) {
                    if(info.type!=StateBase::ArrayInfo::Double)
                        throw std::runtime_error("Requested parameter must be type Double");
                    else if(info.ndim!=1 || info.dim[0]!=6 || info.type!=StateBase::ArrayInfo::Double)
                        throw std::runtime_error("Requested parameter has incorrect shape/type");

                    priv->param_index = idx;
                    break;
                }
            }
        }

        prec->dpvt = priv.release();
        return 0;
    }catch(std::exception& e){
        fprintf(stderr, "%s: init error: %s\n", prec->name, e.what());
        return -1;
    }
}

long orbit_read_tbl(columnarinRecord* prec)
{
    const size_t limit = 127;
    TRY(SimBPMTable) {
        pvd::shared_vector<pvd::uint32> sec, ns;

        sec.reserve(1+priv->sec.size());
        ns.reserve(1+priv->ns.size());

        sec.push_back(priv->sim->last_run.secPastEpoch + POSIX_TIME_AT_EPICS_EPOCH);
        ns.push_back(priv->sim->last_run.nsec);

        for(size_t i=0; i<priv->sec.size() && i<limit; i++) {
            sec.push_back(priv->sec[i]);
        }
        for(size_t i=0; i<priv->ns.size() && i<limit; i++) {
            ns.push_back(priv->ns[i]);
        }

        priv->sec = pvd::freeze(sec);
        priv->ns = pvd::freeze(ns);

        Guard G(priv->sim->lock);

        for(auto& bpm : priv->bpms) {
            StateBase::ArrayInfo info;

            if(!bpm.last->last->getArray(priv->param_index, info)) {
                (void)recGblSetSevrMsg(prec, READ_ALARM, INVALID_ALARM, bpm.colX.c_str());
                continue;
            }

            const double* state=(const double*)info.ptr;

            pvd::shared_vector<double> X, Y;
            X.reserve(1+bpm.X.size());
            X.push_back(state[VectorState::PS_X]);
            Y.reserve(1+bpm.Y.size());
            Y.push_back(state[VectorState::PS_Y]);

            for(size_t i=0; i<bpm.X.size() && i<limit; i++) {
                X.push_back(bpm.X[i]);
            }
            for(size_t i=0; i<bpm.Y.size() && i<limit; i++) {
                Y.push_back(bpm.Y[i]);
            }

            bpm.X = pvd::freeze(X);
            bpm.Y = pvd::freeze(Y);

            columnarin::set_column(prec, bpm.colX.c_str(), pvd::static_shared_vector_cast<const void>(bpm.X));
            columnarin::set_column(prec, bpm.colY.c_str(), pvd::static_shared_vector_cast<const void>(bpm.Y));
        }

        columnarin::set_column(prec, "secondsPastEpoch", pvd::static_shared_vector_cast<const void>(priv->sec));
        columnarin::set_column(prec, "nanoseconds", pvd::static_shared_vector_cast<const void>(priv->ns));

        return 0;
    }CATCH_ALARM()
}

} // namespace

DSET6(columnarin, BPM, orbit_init_ci, Sim::io_aftersim, orbit_read_tbl);
